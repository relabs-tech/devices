// Copyright (c) 2026 Daniel Alarcon Rubio / Relabs Tech
// SPDX-License-Identifier: MIT
// See LICENSE file for full license text

package hmc5983

import (
	"errors"
	"time"

	"periph.io/x/conn/v3/i2c"
)

// I2C register map for HMC5983/HMC5883L.
const (
	regCRA    = 0x00
	regCRB    = 0x01
	regMODE   = 0x02
	regDATA   = 0x03 // X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
	regSTATUS = 0x09
	regIDA    = 0x0A
	regIDB    = 0x0B
	regIDC    = 0x0C
)

// Default I2C address.
const DefaultAddr = 0x1E

// Opts holds initialization options.
//
// ODRHz: output data rate in Hz (maps into CRA bits).
// AvgSamples: sample averaging (1, 2, 4, 8).
// GainCode: 0..7 gain selection (CRB).
// Mode: "continuous" or "single".
// Addr: I2C address, default 0x1E.
//
// When scaling, values are returned in µT×10 to match project conventions.
// Scaling uses typical LSB/Gauss values per gain code and approximates Z by XY
// unless explicitly provided.
type Opts struct {
	ODRHz      int
	AvgSamples int
	GainCode   int
	Mode       string
	Addr       uint16
}

// Dev represents an HMC5983 device.
// Sense returns X,Y,Z values in µT×10.
// Raw counts can be obtained via SenseRaw.
//
// NOTE: HMC5983 outputs data in order X,Z,Y.
type Dev struct {
	dev        i2c.Dev
	lsbPerGaXY int
	lsbPerGaZ  int
}

// New initializes the device.
func New(bus i2c.Bus, opts Opts) (*Dev, error) {
	addr := opts.Addr
	if addr == 0 {
		addr = DefaultAddr
	}
	// Map gain code to LSB/Gauss. Typical values (datasheet):
	// code: XY/Z LSB/Gauss
	gainXY := []int{1370, 1090, 820, 660, 440, 390, 330, 230}
	gainZ := []int{1330, 980, 660, 600, 400, 355, 295, 205}
	gc := opts.GainCode
	if gc < 0 || gc > 7 {
		gc = 1 // default ≈1.3 Gauss
	}

	d := &Dev{
		dev:        i2c.Dev{Addr: addr, Bus: bus},
		lsbPerGaXY: gainXY[gc],
		lsbPerGaZ:  gainZ[gc],
	}

	// Configure CRA: averaging + ODR, normal bias.
	cra := byte(0)
	switch opts.AvgSamples {
	case 8:
		cra |= 0b11 << 5
	case 4:
		cra |= 0b10 << 5
	case 2:
		cra |= 0b01 << 5
	default:
		cra |= 0b00 << 5
	}
	// ODR bits (4..2). Map a few common rates.
	switch opts.ODRHz {
	case 75:
		cra |= 0b110 << 2
	case 30:
		cra |= 0b100 << 2
	case 15:
		cra |= 0b011 << 2
	case 7:
		cra |= 0b010 << 2
	case 3:
		cra |= 0b001 << 2
	default: // 15Hz default
		cra |= 0b011 << 2
	}
	// Bias (bits 1..0): normal (00)
	// Write CRA
	if err := d.writeReg(regCRA, cra); err != nil {
		return nil, err
	}
	// Configure CRB: gain (bits 7..5).
	crb := byte(gc) << 5
	if err := d.writeReg(regCRB, crb); err != nil {
		return nil, err
	}
	// Configure MODE: continuous (0x00) or single (0x01)
	mode := byte(0x00)
	if opts.Mode == "single" {
		mode = 0x01
	}
	if err := d.writeReg(regMODE, mode); err != nil {
		return nil, err
	}
	// Small settle delay.
	time.Sleep(10 * time.Millisecond)
	return d, nil
}

// ID returns the three identity bytes, expected 'H','4','3'.
func (d *Dev) ID() (byte, byte, byte, error) {
	buf := make([]byte, 3)
	if err := d.readRegBlock(regIDA, buf); err != nil {
		return 0, 0, 0, err
	}
	return buf[0], buf[1], buf[2], nil
}

// SenseRaw reads raw counts (X,Z,Y order) and returns X,Y,Z as int16 counts.
func (d *Dev) SenseRaw() (int16, int16, int16, error) {
	data := make([]byte, 6)
	if err := d.readRegBlock(regDATA, data); err != nil {
		return 0, 0, 0, err
	}
	x := int16(data[0])<<8 | int16(data[1])
	z := int16(data[2])<<8 | int16(data[3])
	y := int16(data[4])<<8 | int16(data[5])
	return x, y, z, nil
}

// Sense reads and scales to µT×10 (int16) for X,Y,Z.
func (d *Dev) Sense() (int16, int16, int16, error) {
	rx, ry, rz, err := d.SenseRaw()
	if err != nil {
		return 0, 0, 0, err
	}
	// Convert counts -> Gauss -> µT×10
	// Gauss = counts / LSB_per_Gauss
	// µT = Gauss * 100
	// µT×10 = µT * 10
	gx := float64(rx) / float64(d.lsbPerGaXY)
	gy := float64(ry) / float64(d.lsbPerGaXY)
	gz := float64(rz) / float64(d.lsbPerGaZ)
	ux := int16(gx * 1000.0) // 100 (µT) * 10
	uy := int16(gy * 1000.0)
	uz := int16(gz * 1000.0)
	return ux, uy, uz, nil
}

// Status reads the status register.
func (d *Dev) Status() (byte, error) {
	b := make([]byte, 1)
	if err := d.readRegBlock(regSTATUS, b); err != nil {
		return 0, err
	}
	return b[0], nil
}

func (d *Dev) writeReg(addr byte, val byte) error {
	w := []byte{addr, val}
	if err := d.dev.Tx(w, nil); err != nil {
		return err
	}
	return nil
}

func (d *Dev) readRegBlock(addr byte, out []byte) error {
	if len(out) == 0 {
		return errors.New("readRegBlock: empty buffer")
	}
	w := []byte{addr}
	return d.dev.Tx(w, out)
}

// Convert to physic units if needed (optional helper).
func CountsToMicroTesla10(counts int16, lsbPerGauss int) int16 {
	g := float64(counts) / float64(lsbPerGauss)
	return int16(g * 1000.0) // µT×10
}
