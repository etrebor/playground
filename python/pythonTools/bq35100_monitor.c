/*
 * ATtiny1616 -> BQ35100 I2C monitor (bare-metal, avr-gcc)
 *
 * Pins:  PB0 = SCL, PB1 = SDA        (TWI0 default)
 *        PB2 = ALERT  (input, falling-edge interrupt, pull-up enabled)
 *        PB3 = GE     (gauge enable, output, low at reset)
 *        PA4 = PWR    (output, high at reset)
 *        PB5 = FORCEBAT / PWR2, open-drain style with external pull-up:
 *              input (released, pulled high externally) at reset;
 *              driven low by "~forcebat+"; driven high at the end of ~PWR-;
 *              each PA5 button press toggles it: output high -> input,
 *              anything else (input or output low) -> output high
 *        PA5 = BTN    (input, debounced, active low: N.O. button to GND, 10k pull-up)
 *        PB4 = 5V sense (input, no pull; level must not exceed the ATtiny VDD)
 *        PC3 = 5V sense complement (input, no pull; must always be opposite PB4)
 *        PA1 = USART0 TXD, PA2 = USART0 RXD (alternate pins), 115200 8N1
 * Clock: default 20 MHz / 6 = 3.333 MHz
 *
 * Protocol (USART0):
 *   RX "~GE+"   -> raise GE, wait for gauge ACK, GAUGE_START
 *   RX "~GE-"   -> GAUGE_STOP, wait for G_DONE, drop GE
 *   RX "~PWR+"  -> PA4 high immediately, cancels a pending power-down and
 *                  releases PB5 (unless ~forcebat+ is active)
 *   RX "~PWR-"  -> PA4 low after 10 s (host shutdown time),
 *                  then PB5 switched to output and driven high 10 s after that
 *   RX "~forcebat+" -> PB5 driven low (output low)
 *   RX "~forcebat-" -> PB5 released (input; external pull-up takes it high)
 *   TX "GE ON"  -> gauge enabled and GAUGE_START sent (reply to ~GE+)
 *   TX "GE OFF" -> G_DONE seen (or timed out) and GE dropped (reply to ~GE-)
 *   TX "n.nv"   -> battery voltage every 0.5 s while gauge enabled
 *   TX "5v on"  -> after each voltage read: PB4 high
 *   TX "5v off" -> after each voltage read: PB4 low
 *   TX "status on"  -> after the 5v line: PC3 high
 *   TX "status off" -> after the 5v line: PC3 low
 *   TX "err"    -> after the status line, if PC3 is NOT the opposite level of PB4
 *   TX "ALERT"  -> on each falling edge of ALERT
 *   TX "NOACK"  -> gauge did not answer after ~GE+
 *   TX "I2CERR" -> a voltage read failed
 *   TX "btn+"   -> button pressed  (PA5 pulled low, debounced)
 *   TX "btn-"   -> button released (PA5 back high, debounced)
 *
 *   RX "~cal2"        -> one-time gauge setup for a 2-cell pack measured on VIN:
 *                        unseal if needed, Operation Config A [EXTVCELL]=1
 *                        (ACCUMULATOR mode), Series Cell Count = 2, RESET,
 *                        then report Voltage(). Gauge must not be running.
 *   RX "~vcal<mV>\n"  -> VIN gain calibration: <mV> is the pack voltage
 *                        measured with a meter, e.g. "~vcal6100\n".
 *                        New VIN Gain = old gain x measured / reported.
 *   TX "CAL ..."      -> progress/result lines for ~cal2 and ~vcal
 *
 * Data-flash addresses and procedures: BQ35100 TRM SLUUBH1C.
 * Build: avr-gcc -mmcu=attiny1616 -Os -o bq.elf bq35100_monitor.c
 *
 * CONTROL_STATUS bits per TRM: G_DONE = bit 6, SEC1:SEC0 = bits 14:13.
 */
#define F_CPU 3333333UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

/* Line ending: Unix is LF. Change to "\r" (CR only) or "\r\n" if needed. */
#define EOL "\n"

#define BQ_ADDR         0x55
#define ALERT_bm        PIN2_bm          /* PB2 */
#define GE_bm           PIN3_bm          /* PB3 */
#define PWR_bm          PIN4_bm          /* PA4 */
#define PWR2_bm         PIN5_bm          /* PB5: FORCEBAT / PWR2 */
#define BTN_bm          PIN5_bm          /* PA5 */
#define V5_bm           PIN4_bm          /* PB4: 5 V rail sense */
#define V5N_bm          PIN3_bm          /* PC3: complement of PB4 */
#define BTN_DEBOUNCE_TICKS 3             /* 3 x 10 ms stable = 30 ms */

#define BQ_CONTROL      0x00
#define BQ_VOLTAGE      0x08             /* mV */

#define SUB_CONTROL_STATUS 0x0000
#define SUB_GAUGE_START    0x0011
#define SUB_GAUGE_STOP     0x0012
#define CS_GDONE_bm        0x0040        /* G_DONE (TRM 11.3)  */
#define SUB_RESET          0x0041
#define CS_SEC_gm          0x6000        /* SEC1:SEC0          */
#define CS_SEC_SEALED      0x6000        /* 1,1 = sealed       */

/* Manufacturer access / data flash (TRM ch. 11-13) */
#define BQ_MAC             0x3E          /* ManufacturerAccessControl */
#define BQ_MACDATA         0x40
#define BQ_MACSUM          0x60          /* + 0x61 MACDataLen */
#define DF_VIN_GAIN        0x4010        /* U2, default 29000 */
#define DF_OPCFG_A         0x41B1        /* H1, default 0x80  */
#define DF_SERIES_CELLS    0x4206        /* I1, default 1     */
#define OPCFGA_EXTVCELL    0x40
#define OPCFGA_TEMPS       0x80          /* 1 = external thermistor on TS */
#define OPCFGA_GMSEL_gm    0x03          /* 00 = ACCUMULATOR mode */

#define UNSEAL_KEY1        0x0414        /* TRM default unseal keys */
#define UNSEAL_KEY2        0x3672

#define SWAP16(x) ((uint16_t)((((x) & 0xFFu) << 8) | (((x) >> 8) & 0xFFu)))

#define CAL_SERIES_CELLS   2
/* 1 = force internal temperature sensor (clear TEMPS); 0 = keep current TEMPS */
#define CAL_USE_INTERNAL_TEMP 0

#define PWR_OFF_DELAY_TICKS  20          /* ~PWR- -> PA4 low: 20 x 0.5 s = 10 s */
#define PWR2_OFF_DELAY_TICKS 20          /* PA4 low -> PB5 low: further 10 s    */

static volatile uint8_t f_tick;          /* set every 0.5 s by RTC PIT   */
static volatile uint8_t f_alert;         /* set on ALERT falling edge    */
static volatile char    f_cmd;           /* '+' or '-' from "~GE+/-"     */
static volatile uint8_t pwr_off_ticks;   /* >0: power-down sequence running */
static volatile uint8_t forcebat;        /* 1 = ~forcebat+ holding PB5 low */
static volatile uint8_t f_btn;           /* debounced PA5 state changed  */
static volatile uint8_t btn_state;       /* debounced: 1 = pressed       */
static uint8_t          btn_sent;        /* last level reported to host  */
static volatile char    f_cal;           /* '2' = ~cal2, 'v' = ~vcal     */
static volatile uint16_t cal_mv;         /* measured pack mV for ~vcal   */

/* ---------------- UART ---------------- */
static void uart_init(void)
{
    PORTMUX.CTRLB |= PORTMUX_USART0_bm;                  /* PA1/PA2 */
    PORTA.OUTSET   = PIN1_bm;                            /* idle high */
    PORTA.DIRSET   = PIN1_bm;                            /* TX out */
    PORTA.DIRCLR   = PIN2_bm;                            /* RX in  */
    USART0.BAUD    = (uint16_t)((4UL * F_CPU) / 115200UL); /* 116, -0.2% */
    USART0.CTRLA   = USART_RXCIE_bm;
    USART0.CTRLB   = USART_TXEN_bm | USART_RXEN_bm;
}

static void uart_putc(char c)
{
    while (!(USART0.STATUS & USART_DREIF_bm));
    USART0.TXDATAL = c;
}

static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

/*
 * Command parser: "~GE+", "~GE-", "~PWR+", "~PWR-", "~forcebat+", "~forcebat-",
 *                 "~cal2", "~vcal<mV>\n".
 * '~' always restarts; anything malformed is dropped silently.
 * GE and calibration are handed to main() (they need I2C and can block);
 * PWR is handled here; the delayed power-down is counted in the RTC ISR,
 * so it still fires on time even while main() is blocked.
 */
ISR(USART0_RXC_vect)
{
    static char     buf[8];
    static uint8_t  n = 0xFF;              /* 0xFF = idle, waiting for '~' */
    static uint32_t num;
    char c = USART0.RXDATAL;

    if (c == '~') { n = 0; num = 0; return; }
    if (n == 0xFF) return;

    /* "~vcal" followed by up to 5 digits, ended by CR or LF */
    if (n >= 4 && buf[0] == 'v' && buf[1] == 'c' && buf[2] == 'a' && buf[3] == 'l') {
        if (c >= '0' && c <= '9' && n < 9) {
            num = num * 10 + (uint8_t)(c - '0');
            n++;
            return;
        }
        if ((c == '\r' || c == '\n') && n > 4 && num > 0 && num <= 40000) {
            cal_mv = (uint16_t)num;
            f_cal  = 'v';
        }
        n = 0xFF;
        return;
    }

    if (c == '+' || c == '-') {
        if (n == 2 && buf[0] == 'G' && buf[1] == 'E') {
            f_cmd = c;
        } else if (n == 3 && buf[0] == 'P' && buf[1] == 'W' && buf[2] == 'R') {
            if (c == '+') {
                pwr_off_ticks = 0;             /* cancel pending power-down */
                PORTA.OUTSET  = PWR_bm;
                if (!forcebat)
                    PORTB.DIRCLR = PWR2_bm;    /* release PB5 */
            } else {
                /* PA4 then PB5 drop in the RTC ISR */
                pwr_off_ticks = PWR_OFF_DELAY_TICKS + PWR2_OFF_DELAY_TICKS;
            }
        } else if (n == 8 && buf[0] == 'f' && buf[1] == 'o' && buf[2] == 'r' && buf[3] == 'c' &&
                   buf[4] == 'e' && buf[5] == 'b' && buf[6] == 'a' && buf[7] == 't') {
            if (c == '+') {
                forcebat = 1;
                PORTB.OUTCLR = PWR2_bm;        /* latch low first ...   */
                PORTB.DIRSET = PWR2_bm;        /* ... then drive it     */
            } else {
                forcebat = 0;
                if (!(pwr_off_ticks == 0 && !(PORTA.OUT & PWR_bm)))
                    PORTB.DIRCLR = PWR2_bm;    /* release (input) */
            }
        }
        n = 0xFF;
        return;
    }

    if (n >= sizeof(buf)) { n = 0xFF; return; }   /* too long - not a command */
    buf[n++] = c;

    if (n == 4 && buf[0] == 'c' && buf[1] == 'a' && buf[2] == 'l' && buf[3] == '2') {
        f_cal = '2';
        n = 0xFF;
    }
}

/* ---------------- 0.5 s tick (RTC PIT, internal 32 kHz) ---------------- */
static void tick_init(void)
{
    while (RTC.STATUS > 0);
    RTC.CLKSEL     = RTC_CLKSEL_INT32K_gc;
    while (RTC.PITSTATUS & RTC_CTRLBUSY_bm);
    RTC.PITINTCTRL = RTC_PI_bm;
    RTC.PITCTRLA   = RTC_PERIOD_CYC16384_gc | RTC_PITEN_bm;  /* 16384/32768 = 0.5 s */
}

ISR(RTC_PIT_vect)
{
    RTC.PITINTFLAGS = RTC_PI_bm;
    f_tick = 1;
    if (pwr_off_ticks) {
        pwr_off_ticks--;
        if (pwr_off_ticks == PWR2_OFF_DELAY_TICKS)
            PORTA.OUTCLR = PWR_bm;               /* 10 s after ~PWR- */
        else if (pwr_off_ticks == 0)
        {
            PORTB.OUTSET = PWR2_bm;              /* 10 s after PA4: output high */
            PORTB.DIRSET = PWR2_bm;
        }
    }
}

/* ---------------- ALERT / GE pins ---------------- */
static void gpio_init(void)
{
    PORTB.OUTCLR    = GE_bm;                             /* gauge off at reset */
    PORTB.DIRSET    = GE_bm;
    PORTA.OUTSET    = PWR_bm;                            /* PWR on at reset */
    PORTA.DIRSET    = PWR_bm;
    PORTB.DIRCLR    = PWR2_bm;                           /* PB5 input (released) at reset */
    PORTB.PIN5CTRL  = 0;                                 /* external pull-up */
    PORTB.DIRCLR    = ALERT_bm;
    PORTB.DIRCLR    = V5_bm;                             /* 5 V sense input */
    PORTB.PIN4CTRL  = 0;                                 /* no pull, buffer on */
    PORTC.DIRCLR    = V5N_bm;                            /* PC3 complement input */
    PORTC.PIN3CTRL  = 0;
    PORTB.PIN2CTRL  = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
}

ISR(PORTB_PORT_vect)
{
    uint8_t fl = PORTB.INTFLAGS;
    PORTB.INTFLAGS = fl;
    if (fl & ALERT_bm) f_alert = 1;
}

/* ---------------- Button on PA5 (TCB0 10 ms sampling) ---------------- */
static void btn_init(void)
{
    PORTA.DIRCLR   = BTN_bm;
    PORTA.PIN5CTRL = 0;                  /* external 10k pull-up to 3V3 */
    btn_state = (PORTA.IN & BTN_bm) ? 0 : 1;   /* active low: pressed = 1 */
    btn_sent  = btn_state;               /* no message for the start-up level */

    TCB0.CCMP    = (uint16_t)((F_CPU / 2) / 100 - 1);   /* 10 ms at CLK_PER/2 */
    TCB0.CTRLB   = TCB_CNTMODE_INT_gc;
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CTRLA   = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
}

/* PA5 press: output high -> input; input or output low -> output high */
static void pb5_toggle(void)
{
    if ((PORTB.DIR & PWR2_bm) && (PORTB.OUT & PWR2_bm)) {
        PORTB.DIRCLR = PWR2_bm;            /* release (input) */
    } else {
        PORTB.OUTSET = PWR2_bm;            /* latch high first ... */
        PORTB.DIRSET = PWR2_bm;            /* ... then drive it    */
    }
}

ISR(TCB0_INT_vect)
{
    static uint8_t count;
    uint8_t raw;

    TCB0.INTFLAGS = TCB_CAPT_bm;
    raw = (PORTA.IN & BTN_bm) ? 0 : 1;   /* active low: pressed = 1 */
    if (raw == btn_state) { count = 0; return; }
    if (++count >= BTN_DEBOUNCE_TICKS) {
        count     = 0;
        btn_state = raw;
        f_btn     = 1;
        if (raw)                           /* debounced press */
            pb5_toggle();
    }
}

/*
 * Report button and ALERT events. Called from the main loop and from inside
 * the gauge_on()/gauge_off() wait loops, so events are not held back while
 * the gauge is starting or saving its data.
 */
static void service_async(void)
{
    if (f_btn) {
        f_btn = 0;
        uint8_t s = btn_state;
        if (s != btn_sent) {
            btn_sent = s;
            uart_puts(s ? "btn+" EOL : "btn-" EOL);
        }
    }
    if (f_alert) {
        f_alert = 0;
        uart_puts("ALERT" EOL);
    }
}

/* ---------------- TWI master ---------------- */
static void twi_init(void)
{
    PORTMUX.CTRLB &= ~PORTMUX_TWI0_bm;     /* PB0/PB1 */
    TWI0.MBAUD   = 11;                     /* ~100 kHz @ 3.33 MHz */
    TWI0.MCTRLA  = TWI_ENABLE_bm;
    TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
}

static void twi_stop(void) { TWI0.MCTRLB = TWI_MCMD_STOP_gc; }

static int twi_start(uint8_t addr_rw)
{
    TWI0.MADDR = addr_rw;
    while (!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)));
    if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm)) {
        TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
        return -1;
    }
    if (TWI0.MSTATUS & TWI_RXACK_bm) { twi_stop(); return -2; }
    return 0;
}

static int twi_write(uint8_t d)
{
    TWI0.MDATA = d;
    while (!(TWI0.MSTATUS & TWI_WIF_bm));
    if (TWI0.MSTATUS & (TWI_ARBLOST_bm | TWI_BUSERR_bm)) return -1;
    return (TWI0.MSTATUS & TWI_RXACK_bm) ? -2 : 0;
}

static int bq_read16(uint8_t reg, uint16_t *v)
{
    uint8_t b[2];
    if (twi_start(BQ_ADDR << 1)) return -1;
    if (twi_write(reg)) { twi_stop(); return -1; }
    if (twi_start((BQ_ADDR << 1) | 1)) return -1;
    while (!(TWI0.MSTATUS & TWI_RIF_bm));
    b[0] = TWI0.MDATA;
    TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;                   /* ACK, next */
    while (!(TWI0.MSTATUS & TWI_RIF_bm));
    b[1] = TWI0.MDATA;
    TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;   /* NACK, stop */
    *v = b[0] | ((uint16_t)b[1] << 8);
    return 0;
}

static int bq_write16(uint8_t reg, uint16_t v)
{
    int r = 0;
    if (twi_start(BQ_ADDR << 1)) return -1;
    r |= twi_write(reg);
    r |= twi_write(v & 0xFF);
    r |= twi_write(v >> 8);
    twi_stop();
    return r ? -1 : 0;
}

static int bq_control_status(uint16_t *st)
{
    if (bq_write16(BQ_CONTROL, SUB_CONTROL_STATUS)) return -1;
    _delay_ms(2);
    return bq_read16(BQ_CONTROL, st);
}

/* ---------------- Gauge enable / disable ---------------- */
/* Wait (up to ~1 s) for the gauge to ACK its address. */
static int gauge_wait_ack(void)
{
    for (uint8_t i = 0; i < 100; i++) {
        _delay_ms(10);
        service_async();
        if (twi_start(BQ_ADDR << 1) == 0) {
            twi_stop();
            return 0;
        }
    }
    return -1;
}

/* Raise GE and wait for the gauge; GE is dropped again on failure. */
static int gauge_power_up(void)
{
    PORTB.OUTSET = GE_bm;
    if (gauge_wait_ack() == 0) return 0;
    PORTB.OUTCLR = GE_bm;
    return -1;
}

static int gauge_on(void)
{
    if (gauge_power_up()) return -1;
    bq_write16(BQ_CONTROL, SUB_GAUGE_START);
    return 0;
}

static void gauge_off(void)
{
    uint16_t st = 0;
    bq_write16(BQ_CONTROL, SUB_GAUGE_STOP);
    for (uint8_t i = 0; i < 100; i++) {    /* wait for G_DONE, up to ~10 s */
        _delay_ms(100);
        service_async();
        if (bq_control_status(&st) == 0 && (st & CS_GDONE_bm)) break;
    }
    PORTB.OUTCLR = GE_bm;
}

/* ---------------- Voltage -> "n.nv" ---------------- */
/* Send an unsigned integer in decimal (no library dependency). */
static void uart_put_uint(uint16_t v)
{
    char buf[6];                           /* 65535 + '\0' */
    uint8_t i = sizeof(buf) - 1;

    buf[i] = '\0';
    do {
        buf[--i] = '0' + (v % 10);
        v /= 10;
    } while (v);
    uart_puts(&buf[i]);
}

static void send_5v_state(void)
{
    uint8_t v5  = (PORTB.IN & V5_bm)  ? 1 : 0;
    uint8_t v5n = (PORTC.IN & V5N_bm) ? 1 : 0;

    uart_puts(v5 ? "5v on" EOL : "5v off" EOL);
    uart_puts(v5n ? "status on" EOL : "status off" EOL);
    if (v5 == v5n)                         /* PC3 must be the opposite of PB4 */
        uart_puts("err" EOL);
}

static void send_voltage(void)
{
    uint16_t mv;

    if (bq_read16(BQ_VOLTAGE, &mv)) {
        uart_puts("I2CERR" EOL);
    } else {
        uint16_t tenths = (mv + 50) / 100; /* round to 0.1 V */
        uart_put_uint(tenths / 10);
        uart_putc('.');
        uart_putc('0' + (tenths % 10));
        uart_putc('v');
        uart_puts(EOL);
    }
    send_5v_state();                       /* follows every voltage read */
}

/* ---------------- Gauge setup / calibration (~cal2, ~vcal) ---------------- */
static void uart_put_hex(uint16_t v, uint8_t digits)
{
    static const char hex[] = "0123456789ABCDEF";
    uart_puts("0x");
    while (digits--) uart_putc(hex[(v >> (digits * 4)) & 0x0F]);
}

static void cal_msg(const char *s)
{
    uart_puts("CAL ");
    uart_puts(s);
}

static int bq_read_block(uint8_t reg, uint8_t *buf, uint8_t n)
{
    if (twi_start(BQ_ADDR << 1)) return -1;
    if (twi_write(reg)) { twi_stop(); return -1; }
    if (twi_start((BQ_ADDR << 1) | 1)) return -1;
    for (uint8_t i = 0; i < n; i++) {
        while (!(TWI0.MSTATUS & TWI_RIF_bm));
        buf[i] = TWI0.MDATA;
        if (i < n - 1) TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
        else           TWI0.MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;
    }
    return 0;
}

/*
 * Data flash write (TRM 12.1.1): address LE to 0x3E/0x3F, data big-endian to
 * MACData in the same transaction, then checksum + length as one word to
 * 0x60/0x61. Checksum = ~(sum of address and data bytes), length = n + 4.
 */
static int bq_df_write(uint16_t addr, const uint8_t *d, uint8_t n)
{
    uint8_t sum = (uint8_t)addr + (uint8_t)(addr >> 8);
    int r = 0;

    if (twi_start(BQ_ADDR << 1)) return -1;
    r |= twi_write(BQ_MAC);
    r |= twi_write((uint8_t)addr);
    r |= twi_write((uint8_t)(addr >> 8));
    for (uint8_t i = 0; i < n; i++) {
        r |= twi_write(d[i]);
        sum += d[i];
    }
    twi_stop();
    if (r) return -1;
    _delay_ms(5);

    if (twi_start(BQ_ADDR << 1)) return -1;
    r |= twi_write(BQ_MACSUM);
    r |= twi_write((uint8_t)~sum);
    r |= twi_write((uint8_t)(n + 4));
    twi_stop();
    _delay_ms(100);                        /* flash write time */
    return r ? -1 : 0;
}

/* Data flash read (TRM 12.1.2): set address, then read MACData (big-endian). */
static int bq_df_read(uint16_t addr, uint8_t *d, uint8_t n)
{
    int r = 0;
    if (twi_start(BQ_ADDR << 1)) return -1;
    r |= twi_write(BQ_MAC);
    r |= twi_write((uint8_t)addr);
    r |= twi_write((uint8_t)(addr >> 8));
    twi_stop();
    if (r) return -1;
    _delay_ms(5);
    return bq_read_block(BQ_MACDATA, d, n);
}

/* 0 = ok, -1 = I2C error/NACK during write, -2 = read error, -3 = read-back differs */
static int bq_df_write_u8_verify(uint16_t addr, uint8_t v, uint8_t *rb)
{
    if (bq_df_write(addr, &v, 1)) return -1;
    if (bq_df_read(addr, rb, 1)) return -2;
    return (*rb == v) ? 0 : -3;
}

/* Report why a data-flash write failed, plus CONTROL_STATUS and Voltage(). */
static void cal_write_fail(const char *what, int err, uint8_t rb)
{
    uint16_t st, mv;
    cal_msg("ERR WRITE ");
    uart_puts(what);
    if (err == -1)      uart_puts(" (I2C NACK)");
    else if (err == -2) uart_puts(" (READ-BACK I2C ERROR)");
    else { uart_puts(" (READ BACK "); uart_put_hex(rb, 2); uart_puts(")"); }
    uart_puts(EOL);
    if (bq_control_status(&st) == 0) { cal_msg("STATUS="); uart_put_hex(st, 4); uart_puts(EOL); }
    if (bq_read16(BQ_VOLTAGE, &mv) == 0) { cal_msg("V="); uart_put_uint(mv); uart_puts(" mV" EOL); }
    PORTB.OUTCLR = GE_bm;
}

/* Unseal with the default keys if sealed; tries byte-swapped keys as well. */
static int bq_unseal(void)
{
    static const uint16_t keys[2][2] = {
        { UNSEAL_KEY1, UNSEAL_KEY2 },
        { SWAP16(UNSEAL_KEY1), SWAP16(UNSEAL_KEY2) },
    };
    uint16_t st;

    if (bq_control_status(&st)) return -1;
    cal_msg("STATUS="); uart_put_hex(st, 4); uart_puts(EOL);
    if ((st & CS_SEC_gm) != CS_SEC_SEALED) return 0;

    for (uint8_t k = 0; k < 2; k++) {
        bq_write16(BQ_CONTROL, keys[k][0]);
        _delay_ms(5);
        bq_write16(BQ_CONTROL, keys[k][1]);
        _delay_ms(100);
        if (bq_control_status(&st) == 0 && (st & CS_SEC_gm) != CS_SEC_SEALED) {
            cal_msg("UNSEALED" EOL);
            return 0;
        }
    }
    return -1;
}

/* RESET the gauge, wait for it to come back and for a fresh Voltage(). */
static int bq_reset_and_read_mv(uint16_t *mv)
{
    bq_write16(BQ_CONTROL, SUB_RESET);
    _delay_ms(500);
    if (gauge_wait_ack()) return -1;
    _delay_ms(1500);                       /* Voltage() updates once per second */
    return bq_read16(BQ_VOLTAGE, mv);
}

/* Average of 3 Voltage() readings, 1.1 s apart. */
static int bq_avg_mv(uint16_t *mv)
{
    uint32_t sum = 0;
    uint16_t v;
    for (uint8_t i = 0; i < 3; i++) {
        if (i) _delay_ms(1100);
        if (bq_read16(BQ_VOLTAGE, &v)) return -1;
        sum += v;
    }
    *mv = (uint16_t)(sum / 3);
    return 0;
}

static void cal_fail(const char *why)
{
    cal_msg("ERR ");
    uart_puts(why);
    uart_puts(EOL);
    PORTB.OUTCLR = GE_bm;
}

/* ~cal2: VIN measurement, 2 series cells. */
static void cal_setup_2cell(void)
{
    uint8_t  old, cfg, rb;
    uint16_t mv;
    int      err;

    if (gauge_power_up()) { cal_fail("NOACK"); return; }
    _delay_ms(1500);                       /* let the gauge finish initialising */

    /* Flash writes need Voltage() >= Flash Update OK Voltage (default 2800 mV) */
    if (bq_read16(BQ_VOLTAGE, &mv) == 0) {
        cal_msg("V="); uart_put_uint(mv); uart_puts(" mV before setup" EOL);
    }

    if (bq_unseal()) { cal_fail("SEALED"); return; }

    /* Operation Config A: EXTVCELL = 1, ACCUMULATOR mode */
    if (bq_df_read(DF_OPCFG_A, &old, 1)) { cal_fail("READ OPCFGA"); return; }
    cfg = (old | OPCFGA_EXTVCELL) & (uint8_t)~OPCFGA_GMSEL_gm;
#if CAL_USE_INTERNAL_TEMP
    cfg &= (uint8_t)~OPCFGA_TEMPS;
#endif
    cal_msg("OPCFGA "); uart_put_hex(old, 2); uart_puts("->"); uart_put_hex(cfg, 2); uart_puts(EOL);
    if (cfg != old && (err = bq_df_write_u8_verify(DF_OPCFG_A, cfg, &rb))) {
        cal_write_fail("OPCFGA", err, rb);
        return;
    }

    /* Series Cell Count */
    if (bq_df_read(DF_SERIES_CELLS, &old, 1)) { cal_fail("READ CELLS"); return; }
    cal_msg("CELLS "); uart_put_uint(old); uart_puts("->"); uart_put_uint(CAL_SERIES_CELLS); uart_puts(EOL);
    if (old != CAL_SERIES_CELLS && (err = bq_df_write_u8_verify(DF_SERIES_CELLS, CAL_SERIES_CELLS, &rb))) {
        cal_write_fail("CELLS", err, rb);
        return;
    }

    if (bq_reset_and_read_mv(&mv)) { cal_fail("RESET"); return; }
    cal_msg("V="); uart_put_uint(mv); uart_puts(" mV (uncalibrated)" EOL);
    cal_msg("DONE - now send ~vcal<measured mV>" EOL);
    PORTB.OUTCLR = GE_bm;
}

/* ~vcal<mV>: scale VIN Gain so Voltage() matches the measured pack voltage. */
static void cal_vin_gain(uint16_t actual_mv)
{
    uint8_t  cfg, g[2];
    uint16_t reported, gain_old;
    uint32_t gain_new;

    if (gauge_power_up()) { cal_fail("NOACK"); return; }
    _delay_ms(1500);

    if (bq_unseal()) { cal_fail("SEALED"); return; }
    if (bq_df_read(DF_OPCFG_A, &cfg, 1)) { cal_fail("READ OPCFGA"); return; }
    if (!(cfg & OPCFGA_EXTVCELL)) { cal_fail("EXTVCELL NOT SET - run ~cal2 first"); return; }

    if (bq_avg_mv(&reported)) { cal_fail("READ V"); return; }
    if (reported == 0) { cal_fail("REPORTED 0 mV - check VIN divider"); return; }

    if (bq_df_read(DF_VIN_GAIN, g, 2)) { cal_fail("READ GAIN"); return; }
    gain_old = ((uint16_t)g[0] << 8) | g[1];                /* big-endian */
    gain_new = ((uint32_t)gain_old * actual_mv + reported / 2) / reported;
    if (gain_new == 0 || gain_new > 65535) { cal_fail("GAIN OUT OF RANGE"); return; }

    cal_msg("V="); uart_put_uint(reported); uart_puts(" mV, target "); uart_put_uint(actual_mv); uart_puts(" mV" EOL);
    cal_msg("GAIN "); uart_put_uint(gain_old); uart_puts("->"); uart_put_uint((uint16_t)gain_new); uart_puts(EOL);

    g[0] = (uint8_t)(gain_new >> 8);
    g[1] = (uint8_t)gain_new;
    {
        uint8_t rb[2];
        if (bq_df_write(DF_VIN_GAIN, g, 2) || bq_df_read(DF_VIN_GAIN, rb, 2) ||
            rb[0] != g[0] || rb[1] != g[1]) {
            cal_fail("WRITE GAIN");
            return;
        }
    }

    if (bq_reset_and_read_mv(&reported)) { cal_fail("RESET"); return; }
    cal_msg("V="); uart_put_uint(reported); uart_puts(" mV (calibrated)" EOL);
    cal_msg("DONE" EOL);
    PORTB.OUTCLR = GE_bm;
}

/* ---------------- Main ---------------- */
int main(void)
{
    uint8_t on = 0;

    gpio_init();
    btn_init();
    uart_init();
    twi_init();
    tick_init();
    sei();

    for (;;) {
        char cmd = f_cmd;                  /* single-byte read is atomic */
        f_cmd = 0;

        if (cmd == '+') {
            if (!on && gauge_on() == 0) on = 1;
            uart_puts(on ? "GE ON" EOL : "NOACK" EOL);
        } else if (cmd == '-') {
            if (on) { gauge_off(); on = 0; }
            uart_puts("GE OFF" EOL);
        }

        if (f_cal) {
            char which = f_cal;
            uint16_t mv = cal_mv;
            f_cal = 0;
            if (on) {                      /* flash writes need GA = 0 */
                gauge_off();
                on = 0;
                uart_puts("GE OFF" EOL);
            }
            if (which == '2') cal_setup_2cell();
            else              cal_vin_gain(mv);
        }

        service_async();

        if (f_tick) {
            f_tick = 0;
            if (on) send_voltage();
        }
    }
}
