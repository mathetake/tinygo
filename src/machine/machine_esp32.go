// +build esp32

package machine

import (
	"device/esp"
	"runtime/volatile"
)

const peripheralClock = 80000000 // 80MHz

// CPUFrequency returns the current CPU frequency of the chip.
// Currently it is a fixed frequency but it may allow changing in the future.
func CPUFrequency() uint32 {
	return 160e6 // 160MHz
}

type PinMode uint8

const (
	PinInput     PinMode = 0
	PinOutput            = (1 << 1)
	PinFunction          = (1 << 2)
	PinPullup            = (1 << 3)
	PinPulldown          = (1 << 4)
	PinOpenDrain         = (1 << 5)
	PinFunction0         = 0
	PinFunction1         = (1 << 6)
	PinFunction2         = (1 << 7)
	// PinFunction3         = (1 << 8)
	// PinFunction4         = (1 << 9)
	// PinSpecial = (1 << 13)
	PinInputPullup     = PinInput | PinPullup
	PinInputPulldown   = PinInput | PinPulldown
	PinOutputOpenDrain = PinOutput | PinOpenDrain
	PinInputFunction   = PinInput | PinFunction
	PinInputFunction0  = PinInput | PinFunction0
	PinInputFunction1  = PinInput | PinFunction1
	PinInputFunction2  = PinInput | PinFunction2
	// PinInputFunction3  = PinInput | PinFunction3
	// PinInputFunction4  = PinInput | PinFunction4
	PinOutputFunction  = PinOutput | PinFunction
	PinOutputFunction0 = PinOutput | PinFunction0
	PinOutputFunction1 = PinOutput | PinFunction1
	PinOutputFunction2 = PinOutput | PinFunction2
	// PinOutputFunction3 = PinOutput | PinFunction3
	// PinOutputFunction4 = PinOutput | PinFunction4
)

// Configure this pin with the given configuration.
func (p Pin) Configure(config PinConfig) {
	if config.Mode == PinOutput {
		// Set the 'output enable' bit.
		if p < 32 {
			esp.GPIO.ENABLE_W1TS.Set(1 << p)
		} else {
			esp.GPIO.ENABLE1_W1TS.Set(1 << (p - 32))
		}
	} else {
		// Clear the 'output enable' bit.
		if p < 32 {
			esp.GPIO.ENABLE_W1TC.Set(1 << p)
		} else {
			esp.GPIO.ENABLE1_W1TC.Set(1 << (p - 32))
		}
	}
}

// Set the pin to high or low.
// Warning: only use this on an output pin!
func (p Pin) Set(value bool) {
	if value {
		reg, mask := p.portMaskSet()
		reg.Set(mask)
	} else {
		reg, mask := p.portMaskClear()
		reg.Set(mask)
	}
}

// Return the register and mask to enable a given GPIO pin. This can be used to
// implement bit-banged drivers.
//
// Warning: only use this on an output pin!
func (p Pin) PortMaskSet() (*uint32, uint32) {
	reg, mask := p.portMaskSet()
	return &reg.Reg, mask
}

// Return the register and mask to disable a given GPIO pin. This can be used to
// implement bit-banged drivers.
//
// Warning: only use this on an output pin!
func (p Pin) PortMaskClear() (*uint32, uint32) {
	reg, mask := p.portMaskClear()
	return &reg.Reg, mask
}

func (p Pin) portMaskSet() (*volatile.Register32, uint32) {
	if p < 32 {
		return &esp.GPIO.OUT_W1TS, 1 << p
	} else {
		return &esp.GPIO.OUT1_W1TS, 1 << (p - 32)
	}
}

func (p Pin) portMaskClear() (*volatile.Register32, uint32) {
	if p < 32 {
		return &esp.GPIO.OUT_W1TC, 1 << p
	} else {
		return &esp.GPIO.OUT1_W1TC, 1 << (p - 32)
	}
}

// Get returns the current value of a GPIO pin when the pin is configured as an
// input.
func (p Pin) Get() bool {
	if p < 32 {
		return esp.GPIO.IN.Get()&(1<<p) != 0
	} else {
		return esp.GPIO.IN1.Get()&(1<<(p-32)) != 0
	}
}

var (
	UART0 = UART{Bus: esp.UART0, Buffer: NewRingBuffer()}
	UART1 = UART{Bus: esp.UART1, Buffer: NewRingBuffer()}
	UART2 = UART{Bus: esp.UART2, Buffer: NewRingBuffer()}
)

type UART struct {
	Bus    *esp.UART_Type
	Buffer *RingBuffer
}

func (uart UART) Configure(config UARTConfig) {
	if config.BaudRate == 0 {
		config.BaudRate = 115200
	}
	uart.Bus.CLKDIV.Set(peripheralClock / config.BaudRate)
}

func (uart UART) WriteByte(b byte) error {
	for (uart.Bus.STATUS.Get()>>16)&0xff >= 128 {
		// Read UART_TXFIFO_CNT from the status register, which indicates how
		// many bytes there are in the transmit buffer. Wait until there are
		// less than 128 bytes in this buffer (the default buffer size).
	}
	uart.Bus.TX_FIFO.Set(b)
	return nil
}

// I2C on the ESP32.
type I2C struct {
	Bus *esp.I2C_Type
}

// I2CConfig is used to store config info for I2C.
type I2CConfig struct {
	Frequency uint32
	SCL       Pin
	SDA       Pin
}

const (
	i2cFIFOSize     = 255
	i2cFilterCYCNum = 7
	uint32Max       = 0xFFFFFFFF
)

const (
	i2cCmdRestart = iota //0,        /* I2C restart command */
	i2cCmdWrite          /* I2C write command */
	i2cCmdRead           /* I2C read command */
	i2cCmdStop           /* I2C stop command */
	i2cCmdEnd            /* I2C end command */
)

// Configure is intended to setup the I2C interface.
func (i2c I2C) Configure(config I2CConfig) error {
	// Default I2C bus speed is 100 kHz.
	if config.Frequency == 0 {
		config.Frequency = TWI_FREQ_100KHZ
	}
	// Default I2C pins if not set.
	if config.SDA == 0 && config.SCL == 0 {
		config.SDA = SDA_PIN
		config.SCL = SCL_PIN
	}

	// esp32_gpiowrite(config->scl_pin, 1);
	config.SCL.High()
	// esp32_gpiowrite(config->sda_pin, 1);
	config.SDA.High()

	// esp32_configgpio(config->scl_pin, OUTPUT | OPEN_DRAIN | FUNCTION_2);
	config.SCL.Configure(PinConfig{Mode: PinOutputOpenDrain | PinFunction2})

	// TODO: whatever these do?
	// gpio_matrix_out(config->scl_pin, config->scl_outsig, 0, 0);
	// gpio_matrix_in(config->scl_pin, config->scl_insig, 0);

	// esp32_configgpio(config->sda_pin, INPUT |
	// 								  OUTPUT |
	// 								  OPEN_DRAIN |
	// 								  FUNCTION_2);
	config.SDA.Configure(PinConfig{Mode: PinInput | PinOutputOpenDrain | PinFunction2})

	// TODO: whatever these do?
	// gpio_matrix_out(config->sda_pin, config->sda_outsig, 0, 0);
	// gpio_matrix_in(config->sda_pin, config->sda_insig, 0);

	// modifyreg32(DPORT_PERIP_CLK_EN_REG, 0, config->clk_bit);
	esp.DPORT.PERIP_CLK_EN.SetBits(esp.DPORT_PERIP_CLK_EN_I2C0)

	// modifyreg32(DPORT_PERIP_RST_EN_REG, config->rst_bit, 0);
	esp.DPORT.PERIP_RST_EN.SetBits(esp.DPORT_PERIP_RST_EN_I2C0)

	// esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, 0);
	i2c.Bus.INT_ENA.Set(0)

	// esp32_i2c_set_reg(priv, I2C_INT_CLR_OFFSET, UINT32_MAX);
	i2c.Bus.INT_CLR.Set(uint32Max)

	// esp32_i2c_set_reg(priv, I2C_CTR_OFFSET, I2C_MS_MODE |
	// 										I2C_SCL_FORCE_OUT |
	// 										I2C_SDA_FORCE_OUT);
	i2c.Bus.CTR.Set(esp.I2C_CTR_MS_MODE | esp.I2C_CTR_SCL_FORCE_OUT | esp.I2C_CTR_SDA_FORCE_OUT)

	// esp32_i2c_reset_reg_bits(priv, I2C_FIFO_CONF_OFFSET,
	// 						 I2C_NONFIFO_EN_M);
	i2c.Bus.FIFO_CONF.Set(esp.I2C_FIFO_CONF_NONFIFO_EN)

	i2c.resetFIFO()

	// esp32_i2c_set_reg(priv, I2C_SCL_FILTER_CFG_OFFSET, I2C_SCL_FILTER_EN_M |
	// 	I2C_FILTER_CYC_NUM_DEF);
	i2c.Bus.SCL_FILTER_CFG.Set(esp.I2C_SCL_FILTER_CFG_SCL_FILTER_EN | i2cFilterCYCNum)

	// esp32_i2c_set_reg(priv, I2C_SDA_FILTER_CFG_OFFSET, I2C_SDA_FILTER_EN_M |
	// 	I2C_FILTER_CYC_NUM_DEF);
	i2c.Bus.SDA_FILTER_CFG.Set(esp.I2C_SDA_FILTER_CFG_SDA_FILTER_EN | i2cFilterCYCNum)

	i2c.SetBaudRate(config.Frequency)
	return nil
}

// SetBaudRate sets the communication speed for the I2C.
func (i2c I2C) SetBaudRate(br uint32) {
	// uint32_t half_cycles = APB_CLK_FREQ / clk_freq / 2;
	hc := peripheralClock / br / 2

	// uint32_t timeout_cycles = half_cycles * 20;
	tc := hc * 20

	// esp32_i2c_set_reg(priv, I2C_SCL_LOW_PERIOD_OFFSET, half_cycles);
	i2c.Bus.SCL_LOW_PERIOD.Set(hc)

	// esp32_i2c_set_reg(priv, I2C_SCL_HIGH_PERIOD_OFFSET, half_cycles);
	i2c.Bus.SCL_HIGH_PERIOD.Set(hc)

	// esp32_i2c_set_reg(priv, I2C_SDA_HOLD_OFFSET, half_cycles / 2);
	i2c.Bus.SDA_HOLD.Set(hc >> 1)

	// esp32_i2c_set_reg(priv, I2C_SDA_SAMPLE_OFFSET, half_cycles / 2);
	i2c.Bus.SDA_SAMPLE.Set(hc >> 1)

	// esp32_i2c_set_reg(priv, I2C_SCL_RSTART_SETUP_OFFSET, half_cycles);
	i2c.Bus.SCL_RSTART_SETUP.Set(hc)

	// esp32_i2c_set_reg(priv, I2C_SCL_STOP_SETUP_OFFSET, half_cycles);
	i2c.Bus.SCL_STOP_SETUP.Set(hc)

	// esp32_i2c_set_reg(priv, I2C_SCL_START_HOLD_OFFSET, half_cycles);
	i2c.Bus.SCL_START_HOLD.Set(hc)

	// esp32_i2c_set_reg(priv, I2C_SCL_STOP_HOLD_OFFSET, half_cycles);
	i2c.Bus.SCL_STOP_HOLD.Set(hc)

	// esp32_i2c_set_reg(priv, I2C_TO_OFFSET, timeout_cycles);
	i2c.Bus.TO.Set(tc)
}

// Tx does a single I2C transaction at the specified address.
// It clocks out the given address, writes the bytes in w, reads back len(r)
// bytes and stores them in r, and generates a stop condition on the bus.
func (i2c I2C) Tx(addr uint16, w, r []byte) error {
	return nil
}

// WriteByte writes a single byte to the I2C bus.
func (i2c I2C) WriteByte(data byte) error {
	return nil
}

func i2CBaseCmd(cmd, checkAck int) uint32 {
	return uint32((cmd << 11) | (checkAck << 8))
}

func i2CSendCmd(cmd, checkAck, size int) uint32 {
	return uint32((cmd << 11) | (checkAck << 8) + size)
}

// sendAddress sends the address and start signal
func (i2c I2C) sendAddress(address uint16, write bool) error {
	return nil
}

func (i2c I2C) signalStart(size int) error {
	// 	esp32_i2c_set_reg(priv, I2C_COMD0_OFFSET,
	// 		I2C_BASE_CMD(I2C_CMD_RESTART, 0));
	i2c.Bus.COMD0.Set(i2CBaseCmd(i2cCmdRestart, 0))

	// esp32_i2c_set_reg(priv, I2C_COMD1_OFFSET,
	// 		I2C_SEND_CMD(I2C_CMD_WRITE, 1, 1));
	i2c.Bus.COMD1.Set(i2CSendCmd(i2cCmdWrite, 1, size))

	// esp32_i2c_set_reg(priv, I2C_COMD2_OFFSET,
	// 		I2C_BASE_CMD(I2C_CMD_END, 0));
	i2c.Bus.COMD2.Set(i2CBaseCmd(i2cCmdEnd, 0))

	// esp32_i2c_set_reg(priv, I2C_DATA_OFFSET, (msg->addr << 1) |
	// 							   (msg->flags & I2C_M_READ));
	//i2c.Bus.DATA.Set()

	// esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, I2C_END_DETECT_INT_ENA |
	// 								  I2C_INT_ERR_EN_BITS);
	i2c.Bus.INT_ENA.Set(esp.I2C_INT_ENA_END_DETECT_INT_ENA |
		esp.I2C_INT_ENA_ACK_ERR_INT_ENA)

	// esp32_i2c_set_reg_bits(priv, I2C_CTR_OFFSET, I2C_TRANS_START_M);
	i2c.Bus.CTR.SetBits(esp.I2C_INT_ENA_TRANS_START_INT_ENA)

	return nil
}

func (i2c I2C) signalStop() error {
	// esp32_i2c_set_reg(priv, I2C_COMD0_OFFSET, I2C_BASE_CMD(I2C_CMD_STOP, 0));
	i2c.Bus.COMD0.Set(i2CBaseCmd(i2cCmdStop, 0))

	// esp32_i2c_set_reg(priv, I2C_INT_ENA_OFFSET, I2C_TRANS_COMPLETE_INT_ENA |
	//                                               I2C_INT_ERR_EN_BITS);
	i2c.Bus.INT_ENA.Set(esp.I2C_INT_ENA_TRANS_COMPLETE_INT_ENA |
		esp.I2C_INT_ENA_ACK_ERR_INT_ENA)

	// esp32_i2c_set_reg_bits(priv, I2C_CTR_OFFSET, I2C_TRANS_START_M);
	i2c.Bus.CTR.SetBits(esp.I2C_INT_ENA_TRANS_START_INT_ENA)

	return nil
}

func (i2c I2C) signalRead() error {
	//uint32_t cmd = esp32_i2c_get_reg(priv, I2C_COMD0_OFFSET);
	//uint8_t n = cmd & 0xff;

	return nil
}

func (i2c I2C) readByte() byte {
	// esp32_i2c_get_reg(priv,
	// 	I2C_DATA_OFFSET);
	return byte(i2c.Bus.DATA.Get())
}

func (i2c I2C) resetFIFO() {
	// esp32_i2c_set_reg_bits(priv, I2C_FIFO_CONF_OFFSET, I2C_TX_FIFO_RST);
	i2c.Bus.FIFO_CONF.SetBits(esp.I2C_FIFO_CONF_TX_FIFO_RST)

	// esp32_i2c_reset_reg_bits(priv, I2C_FIFO_CONF_OFFSET, I2C_TX_FIFO_RST);
	i2c.Bus.FIFO_CONF.ClearBits(esp.I2C_FIFO_CONF_TX_FIFO_RST)

	// esp32_i2c_set_reg_bits(priv, I2C_FIFO_CONF_OFFSET, I2C_RX_FIFO_RST);
	i2c.Bus.FIFO_CONF.SetBits(esp.I2C_FIFO_CONF_RX_FIFO_RST)

	// esp32_i2c_reset_reg_bits(priv, I2C_FIFO_CONF_OFFSET, I2C_RX_FIFO_RST);
	i2c.Bus.FIFO_CONF.ClearBits(esp.I2C_FIFO_CONF_RX_FIFO_RST)
}
