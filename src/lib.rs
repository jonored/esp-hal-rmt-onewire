#![no_std]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use esp_hal as hal;

use embassy_futures::join::*;
use hal::gpio::*;
use hal::peripheral::*;
use hal::rmt::{asynch::*, PulseCode, RxChannelConfig, TxChannelConfig, *};

pub struct OneWire<R: RxChannelAsync, T: TxChannelAsync> {
    rx: R,
    tx: T,
}

impl<R: RxChannelAsync, T: TxChannelAsync> OneWire<R, T> {
    pub fn new<
        'd,
        Tx: TxChannelCreatorAsync<'d, T, P>,
        Rx: RxChannelCreatorAsync<'d, R, P>,
        P: 'd + InputPin + OutputPin + Peripheral<P = P>,
    >(
        txcc: Tx,
        rxcc: Rx,
        mut pin: P,
    ) -> OneWire<R, T> {
        let rx_config = RxChannelConfig {
            clk_divider: 80,
            idle_threshold: 1000,
            filter_threshold: 10,
            carrier_modulation: false,
            ..RxChannelConfig::default()
        };
        let tx_config = TxChannelConfig {
            clk_divider: 80,
            carrier_modulation: false,
            ..TxChannelConfig::default()
        };

        let tx = txcc
            .configure(unsafe { pin.clone_unchecked() }, tx_config)
            .unwrap();
        let rx = rxcc
            .configure(unsafe { pin.clone_unchecked() }, rx_config)
            .unwrap();
        pin.internal_pull_up(true);
        pin.enable_output(true);
        pin.enable_open_drain(true);
        pin.enable_input(true);
        pin.set_drive_strength(hal::gpio::DriveStrength::I40mA);
        pin.connect_input_to_peripheral_with_options(hal::gpio::InputSignal::RMT_SIG_0, true, true);
        pin.connect_peripheral_to_output_with_options(
            hal::gpio::OutputSignal::RMT_SIG_0,
            true,
            false,
            false,
            true,
        );

        OneWire { rx, tx }
    }

    pub async fn reset(&mut self) -> bool {
        let data = [
            PulseCode {
                level1: false,
                length1: 60,
                level2: true,
                length2: 600,
            },
            PulseCode {
                level1: false,
                length1: 600,
                level2: false,
                length2: 0,
            },
            PulseCode::default(),
        ];
        let mut indata = [PulseCode::default(); 10];

        // TODO: error handling
        let _res = self.send_and_receive(&mut indata, &data).await;

        indata[0].length1 > 0
            && indata[0].length2 > 0
            && indata[1].length1 > 100
            && indata[1].length1 < 200
    }

    pub async fn send_and_receive(
        &mut self,
        indata: &mut [PulseCode],
        data: &[PulseCode],
    ) -> Result<(), esp_hal::rmt::Error> {
        // This relies on join polling in order to set up the rx & tx registers, which is not strictly documented behavior.
        let res = join(self.rx.receive(indata), self.tx.transmit(data)).await;
        res.0.and(res.1)
    }

    const ZERO_BIT_LEN: u16 = 70;
    const ONE_BIT_LEN: u16 = 3;

    pub fn encode_bit(bit: bool) -> PulseCode {
        if bit {
            PulseCode {
                length1: Self::ONE_BIT_LEN,
                level1: true,
                length2: Self::ZERO_BIT_LEN,
                level2: false,
            }
        } else {
            PulseCode {
                length1: Self::ZERO_BIT_LEN,
                level1: true,
                length2: Self::ONE_BIT_LEN,
                level2: false,
            }
        }
    }

    pub fn decode_bit(code: PulseCode) -> bool {
        let len = code.length1;
        if len < 20 {
            true
        } else {
            false
        }
    }

    pub async fn exchange_byte(&mut self, byte: u8) -> u8 {
        let mut data = [PulseCode::default(); 10];
        let mut indata = [PulseCode::default(); 10];
        for n in 0..8 {
            data[n] = Self::encode_bit(0 != byte & 1 << n);
        }
        // TODO: error handling
        let _res = self.send_and_receive(&mut indata, &data).await;
        let mut res: u8 = 0;
        for n in 0..8 {
            if Self::decode_bit(indata[n]) {
                res |= 1 << n;
            }
        }
        res
    }

    pub async fn send_byte(&mut self, byte: u8) {
        let mut data = [PulseCode::default(); 10];
        for n in 0..8 {
            data[n] = Self::encode_bit(0 != byte & 1 << n);
        }
        // TODO: error handling
        let _res = self.tx.transmit(&data).await;
    }

    pub async fn exchange_bits<const N: usize>(&mut self, bits: [bool; N]) -> [bool; N]
    where
        [(); N + 1]:,
    {
        let mut data = [PulseCode::default(); N + 1];
        let mut indata = [PulseCode::default(); N + 1];
        for n in 0..N {
            data[n] = Self::encode_bit(bits[n]);
        }
        // TODO: error handling
        let _res = self.send_and_receive(&mut indata, &data).await;
        let mut res: [bool; N] = [false; N];
        for n in 0..N {
            res[n] = Self::decode_bit(indata[n]);
        }
        res
    }

    pub async fn send_u64(&mut self, val: u64) {
        for byte in val.to_le_bytes() {
            self.send_byte(byte).await;
        }
    }

    pub async fn send_address(&mut self, val: Address) {
        self.send_u64(val.0).await
    }
}

pub struct Address(u64);

impl core::fmt::Debug for Address {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        core::write!(f, "{:X?}", self.0.to_le_bytes())
    }
}

pub struct Search {
    command: u8,
    address: u64,
    last_discrepancy: Option<usize>,
    complete: bool,
}

#[derive(Debug)]
pub enum SearchError {
    SearchComplete,
    NoDevicesPresent,
}

impl Search {
    pub fn new() -> Search {
        Search {
            command: 0xF0,
            address: 0, // [false; 64],
            last_discrepancy: None,
            complete: false,
        }
    }
    pub fn new_alarm() -> Search {
        Search {
            command: 0xEC,
            address: 0, // [false; 64],
            last_discrepancy: None,
            complete: false,
        }
    }
    pub async fn next<R: RxChannelAsync, T: TxChannelAsync>(
        &mut self,
        ow: &mut OneWire<R, T>,
    ) -> Result<Address, SearchError> {
        if self.complete {
            return Err(SearchError::SearchComplete);
        }
        let have_devices = ow.reset().await;
        let mut last_zero = None;
        ow.send_byte(self.command).await;
        if have_devices {
            for id_bit_number in 0..64 {
                let id_bits = ow.exchange_bits([true, true]).await;
                let search_direction = match id_bits {
                    [false, true] => false,
                    [true, false] => true,
                    [true, true] => {
                        return Err(SearchError::NoDevicesPresent);
                    }
                    [false, false] => {
                        if self.last_discrepancy == Some(id_bit_number) {
                            true
                        } else if Some(id_bit_number) > self.last_discrepancy {
                            last_zero = Some(id_bit_number);
                            false
                        } else {
                            self.address & (1 << id_bit_number) != 0
                        }
                    }
                };
                if search_direction {
                    self.address |= 1 << id_bit_number;
                } else {
                    self.address &= !(1 << id_bit_number);
                }
                ow.exchange_bits([search_direction]).await;
            }
            self.last_discrepancy = last_zero;
            self.complete = last_zero.is_none();
            Ok(Address(self.address))
        } else {
            Err(SearchError::NoDevicesPresent)
        }
    }
}
