#![no_std]
use embedded_hal::i2c::I2c;


fn crc4(n_prom: &mut [u16; 8]) -> u8 {
    let mut n_rem: u16 = 0;

    n_prom[0] &= 0x0FFF;
    n_prom[7] = 0;

    for cnt in 0..16 {
        if cnt % 2 == 1 {
            n_rem ^= n_prom[cnt >> 1] & 0x00FF;
        } else {
            n_rem ^= n_prom[cnt >> 1] >> 8;
        }
        for _ in 0..8 {
            if (n_rem & 0x8000) != 0 {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem <<= 1;
            }
        }
    }
    ((n_rem >> 12) & 0x000F) as u8
}

pub struct SensorData {
    pub temperature: i32, // 0.01 deg_C
    pub pressure: i32, // 0.01 mBar
}

pub struct Ms5837_02ba {
    pressure_sensitivity: u16,              // SENS
    pressure_offset: u16,                   // OFF
    temp_coef_pressure_sensitivity: u16,    // TCS
    temp_coef_pressure_offset: u16,         // TCO
    reference_temperature: u16,             // T_REF
    temp_coef_temperature: u16,             // TEMPSENS
}

impl Ms5837_02ba {
    const ADDRESS: u8 = 0x76;
    const RESET: u8 = 0b0001_1110;
    const PROM_READ: u8 = 0b1010_0000;
    const D1_OSR2048: u8 = 0b0100_0110;
    const D2_OSR2048: u8 = 0b0101_0110;
    const ADC_READ: u8 = 0b0000_0000;

    pub fn new<I: I2c>(i2c: &mut I) -> Result<Ms5837_02ba, I::Error> {
        // Reset the system so it loads the PROM
        i2c.write(Self::ADDRESS, &[Self::RESET])?;
        let mut entire_prom = [0u16; 8];
        let mut buffer = [0; 2];
        // First address of PROM CRC[16:13] Version[12:5] Factory defined [4:0]
        i2c.write_read(Self::ADDRESS, &[Self::PROM_READ | (0  << 1)], &mut buffer)?;
        entire_prom[0] = u16::from_be_bytes(buffer);
        let crc = buffer[0] >> 4;
        i2c.write_read(Self::ADDRESS, &[Self::PROM_READ | (1 << 1)], &mut buffer)?;
        entire_prom[1] = u16::from_be_bytes(buffer);
        let pressure_sensitivity = entire_prom[1];
        i2c.write_read(Self::ADDRESS, &[Self::PROM_READ | (2 << 1)], &mut buffer)?;
        entire_prom[2] = u16::from_be_bytes(buffer);
        let pressure_offset = entire_prom[2];
        i2c.write_read(Self::ADDRESS, &[Self::PROM_READ | (3 << 1)], &mut buffer)?;
        entire_prom[3] = u16::from_be_bytes(buffer);
        let temp_coef_pressure_sensitivity = entire_prom[3];
        i2c.write_read(Self::ADDRESS, &[Self::PROM_READ | (4 << 1)], &mut buffer)?;
        entire_prom[4] = u16::from_be_bytes(buffer);
        let temp_coef_pressure_offset = entire_prom[4];
        i2c.write_read(Self::ADDRESS, &[Self::PROM_READ | (5 << 1)], &mut buffer)?;
        entire_prom[5] = u16::from_be_bytes(buffer);
        let reference_temperature = entire_prom[5];
        i2c.write_read(Self::ADDRESS, &[Self::PROM_READ | (6 << 1)], &mut buffer)?;
        entire_prom[6] = u16::from_be_bytes(buffer);
        let temp_coef_temperature = entire_prom[6];
        let calculated_crc = crc4(&mut entire_prom);
        assert_eq!(crc, calculated_crc);

        Ok(Ms5837_02ba {
            pressure_sensitivity,
            pressure_offset,
            temp_coef_pressure_sensitivity,
            temp_coef_pressure_offset,
            reference_temperature,
            temp_coef_temperature,
        })
    }
    const READ_TIMEOUT_ITER: u8 = 200;
    pub fn read<I: I2c>(&self, i2c: &mut I) -> Result<Option<SensorData>, (I::Error, usize)> {
        let mut buffer = [0u8; 3];
        i2c.write(Self::ADDRESS, &[Self::D1_OSR2048]).map_err(|e| (e, 0))?;

        let mut iteration_count = 0;
        while i2c.write(Self::ADDRESS, &[Self::ADC_READ]).is_err() {
            iteration_count += 1;
            if iteration_count >= Self::READ_TIMEOUT_ITER {
                return Ok(None);
            }
        }
        i2c.read(Self::ADDRESS, &mut buffer).map_err(|e| (e, 3))?;
        let digital_pressure = (buffer[0] as u32) << 16 | (buffer[1] as u32) << 8 | (buffer[2] as u32);

        i2c.write(Self::ADDRESS, &[Self::D2_OSR2048]).map_err(|e| (e, 4))?;
        let mut iteration_count = 0;
        while i2c.write(Self::ADDRESS, &[Self::ADC_READ]).is_err() {
            iteration_count += 1;
            if iteration_count >= Self::READ_TIMEOUT_ITER {
                return Ok(None);
            }
        }
        i2c.read(Self::ADDRESS, &mut buffer).map_err(|e| (e, 6))?;

        let digital_temperature = (buffer[0] as u32) << 16 | (buffer[1] as u32) << 8 | (buffer[2] as u32);
        let temp_difference: i32 = (digital_temperature as i32) - ((self.reference_temperature as i32) << 8);
        let actual_temperature =2000 + ((temp_difference as i64 * self.temp_coef_temperature as i64) >> 23) as i32;


        let offset = ((self.pressure_offset as i64) << 17) + (((self.temp_coef_pressure_offset as i64) * (temp_difference as i64)) >> 6);
        let sensitivity = ((self.pressure_sensitivity as i64) << 16) + (((self.temp_coef_pressure_sensitivity as i64) * (temp_difference as i64)) >> 7);
        let temperature_compensated_pressure = ((((digital_pressure as i64) * sensitivity) >> 21) - offset) as i32 >> 15;

        // Apply second order correction
        if actual_temperature < 2000 {
            let t_i = ((11 * (temp_difference as i64).pow(2)) >> 35) as i32;
            let d_t = actual_temperature - 2000;
            let off_i = (31 * (d_t as i64).pow(2)) >> 3;
            let sens_i = (63 * (d_t as i64).pow(2)) >> 5;
            let off2: i64 = offset - off_i;
            let sens2 = sensitivity - sens_i;
            let temp2 = actual_temperature - t_i;
            let p2 = ((((digital_pressure as i64) * sens2) >> 21) - off2) as i32 >> 15;
            return Ok(Some(SensorData {
                temperature: temp2,
                pressure: p2,
            }))
        }
        Ok(Some(SensorData {
            temperature: actual_temperature,
            pressure: temperature_compensated_pressure,
        }))
    }
}