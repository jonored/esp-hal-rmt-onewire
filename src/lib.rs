#![no_std]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use esp_hal as hal;

use hal::gpio::{InputPin,OutputPin,Pull,Level, Flex};
use hal::peripheral::*;
use hal::rmt::{RxChannelConfig, TxChannelConfig, RxChannelAsync, TxChannelAsync, PulseCode, TxChannelCreatorAsync, RxChannelCreatorAsync};
use hal::gpio::interconnect::*;
use embassy_futures::select::*;

pub struct OneWire<R: RxChannelAsync, T: TxChannelAsync> {
    rx: R,
    tx: T,
    input: InputSignal,
}

impl<'d, R: RxChannelAsync, T: TxChannelAsync> OneWire<R, T> {
    pub fn new<
        Tx: TxChannelCreatorAsync<'d, T, OutputSignal>,
        Rx: RxChannelCreatorAsync<'d, R, InputSignal>,
        P: 'd + InputPin + OutputPin + Peripheral<P = P>,
    >(
        txcc: Tx,
        rxcc: Rx,
        pin: P,
    ) -> Result<OneWire<R, T>, Error> {
        let rx_config = RxChannelConfig::default()
            .with_clk_divider(80)
            .with_idle_threshold(1000)
            .with_filter_threshold(10)
            .with_carrier_modulation(false);
        let tx_config = TxChannelConfig::default()
            .with_clk_divider(80)
            .with_carrier_modulation(false);

        let mut pin : Flex = Flex::new(pin.into());

        // Clone the pin so we can still overwrite the (wrong) config set by the rmt tx and rx driver setup:
        let (insig, outsig) = unsafe { pin.clone_unchecked() }.split();

        let tx = txcc
            .configure(outsig.inverted(), tx_config).map_err(Error::SendError)?;
        let rx = rxcc
            .configure(insig.inverted(), rx_config).map_err(Error::ReceiveError)?;
       
        // Then we set the pin up correctly: 
        pin.set_as_open_drain(Pull::Up);
        pin.set_drive_strength(hal::gpio::DriveStrength::_40mA);
        pin.enable_input(true);

        // and connect the physical peripherals back up to the GPIO matrix:
        let (input, outsig) = pin.split();
        R::input_signal().connect_to(input.clone().inverted());
        T::output_signal().connect_to(outsig.inverted());

        Ok(OneWire { rx, tx, input })
    }

    pub async fn reset(&mut self) -> Result<bool, Error> {
        let data = [
            PulseCode::new(Level::Low, 60, Level::High, 600),
            PulseCode::new(Level::Low, 600, Level::Low, 0),
            PulseCode::empty(),
        ];
        let mut indata = [PulseCode::empty(); 10];

        let _res = self.send_and_receive(&mut indata, &data).await?;

        Ok(indata[0].length1() > 0
            && indata[0].length2() > 0
            && indata[1].length1() > 100
            && indata[1].length1() < 200
        )
    }

    pub async fn send_and_receive(
        &mut self,
        indata: &mut [u32],
        data: &[u32],
    ) -> Result<(), Error> {
        let delay = [PulseCode::new(Level::Low, 30000, Level::Low, 0)]; // timeout delay for 30ms using the RMT tx peripheral.
        if self.input.level() == Level::Low {
            Err(Error::InputNotHigh)?;
        }
        // This relies on select polling in order to set up the rx & tx registers, which is not strictly documented behavior.
        let res = select(self.rx.receive(indata), async { let r = self.tx.transmit(data).await; let _ = self.tx.transmit(&delay).await; r } ).await;
        T::stop(); // Need to use the internal interface to stop our TX-based timeout here.
        match res {
           Either::First(Ok(r)) => Ok(r),
           Either::First(Err(r)) => Err(Error::ReceiveError(r)),
           Either::Second(Ok(())) => Err(Error::ReceiveTimedOut),
           Either::Second(Err(e)) => Err(Error::SendError(e)),
        }
    }

    const ZERO_BIT_LEN: u16 = 70;
    const ONE_BIT_LEN: u16 = 3;

    pub fn encode_bit(bit: bool) -> u32 {
        if bit {
            PulseCode::new(Level::High, Self::ONE_BIT_LEN, Level::Low, Self::ZERO_BIT_LEN)
        } else {
            PulseCode::new(Level::High, Self::ZERO_BIT_LEN, Level::Low, Self::ONE_BIT_LEN)
        }
    }

    pub fn decode_bit(code: u32) -> bool {
        let len = code.length1();
        if len < 20 {
            true
        } else {
            false
        }
    }

    pub async fn exchange_byte(&mut self, byte: u8) -> Result<u8, Error> {
        let mut data = [PulseCode::empty(); 10];
        let mut indata = [PulseCode::empty(); 10];
        for n in 0..8 {
            data[n] = Self::encode_bit(0 != byte & 1 << n);
        }
        let _res = self.send_and_receive(&mut indata, &data).await?;
        let mut res: u8 = 0;
        for n in 0..8 {
            if Self::decode_bit(indata[n]) {
                res |= 1 << n;
            }
        }
        Ok(res)
    }

    pub async fn send_byte(&mut self, byte: u8) -> Result<(), Error> {
        let mut data = [PulseCode::empty(); 10];
        for n in 0..8 {
            data[n] = Self::encode_bit(0 != byte & 1 << n);
        }
        let _res = self.tx.transmit(&data).await?;
        Ok(())
    }

    pub async fn exchange_bits<const N: usize>(&mut self, bits: [bool; N]) -> Result<[bool; N], Error>
    where
        [(); N + 1]:,
    {
        let mut data = [PulseCode::empty(); N + 1];
        let mut indata = [PulseCode::empty(); N + 1];
        for n in 0..N {
            data[n] = Self::encode_bit(bits[n]);
        }
        let _res = self.send_and_receive(&mut indata, &data).await?;
        let mut res: [bool; N] = [false; N];
        for n in 0..N {
            res[n] = Self::decode_bit(indata[n]);
        }
        Ok(res)
    }

    pub async fn send_u64(&mut self, val: u64) -> Result<(), Error>{
        for byte in val.to_le_bytes() {
            self.send_byte(byte).await?;
        }
        Ok(())
    }

    pub async fn send_address(&mut self, val: Address) -> Result<(), Error> {
        self.send_u64(val.0).await
    }
}

#[derive(Debug)]
pub enum Error {
    InputNotHigh,
    ReceiveTimedOut,
    ReceiveError(esp_hal::rmt::Error),
    SendError(esp_hal::rmt::Error),
}

impl From<esp_hal::rmt::Error> for Error {
    fn from(e : esp_hal::rmt::Error) -> Error {
        Error::SendError(e)
    }
}

#[derive(PartialEq, Eq, Clone, Copy)]
pub struct Address(u64);

impl core::fmt::Debug for Address {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        core::write!(f, "{:X?}", self.0.to_le_bytes())
    }
}

impl core::fmt::Display for Address {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        for k in self.0.to_le_bytes() {
		core::write!(f, "{:X}", k)?;
        }
        Ok(())
    }
}

pub struct Search {
    command: u8,
    address: u64,
    #[cfg(feature = "search-masks")]
    address_mask: u64,
    last_discrepancy: Option<usize>,
    complete: bool,
}

#[derive(Debug)]
pub enum SearchError {
    SearchComplete,
    NoDevicesPresent,
    BusError(Error),
}

impl From<Error> for SearchError {
    fn from(e: Error) -> SearchError {
       SearchError::BusError(e)
    }
}

impl Search {
    pub fn new() -> Search {
        Search {
            command: 0xF0,
            address: 0,
            #[cfg(feature = "search-masks")]
            address_mask: 0,
            last_discrepancy: None,
            complete: false,
        }
    }
    pub fn new_alarm() -> Search {
        Search {
            command: 0xEC,
            address: 0,
            #[cfg(feature = "search-masks")]
            address_mask: 0,
            last_discrepancy: None,
            complete: false,
        }
    }
    #[cfg(feature = "search-masks")]
    pub fn new_with_mask(fixed_bits: u64, bit_mask: u64) {
        Search {
            command: 0xEC,
            address: fixed_bits,
            address_mask: bit_mask,
            last_discrepancy: None,
            complete: false,
        }
    }
    pub async fn next<'d, R: RxChannelAsync, T: TxChannelAsync>(
        &mut self,
        ow: &mut OneWire<R, T>,
    ) -> Result<Address, SearchError> {
        if self.complete {
            return Err(SearchError::SearchComplete);
        }
        let have_devices = ow.reset().await?;
        let mut last_zero = None;
        ow.send_byte(self.command).await?;
        if have_devices {
            for id_bit_number in 0..64 {
                let id_bits = ow.exchange_bits([true, true]).await?;
                let search_direction = match id_bits {
                    #[cfg(feature = "search-masks")]
                    _ if address_mask & (1 << id_bit_number) != 0 => {
                        address & (1 << id_bit_number) != 0
                    }
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
                ow.exchange_bits([search_direction]).await?;
            }
            self.last_discrepancy = last_zero;
            self.complete = last_zero.is_none();
            Ok(Address(self.address))
        } else {
            Err(SearchError::NoDevicesPresent)
        }
    }
}
