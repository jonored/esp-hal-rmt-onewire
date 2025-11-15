#![no_std]

use embassy_futures::select::*;
use esp_hal::{
    Async,
    gpio::{
        DriveMode, DriveStrength, Flex, InputConfig, Level, OutputConfig, Pin, Pull,
        interconnect::*,
    },
    rmt::{
        Channel, PulseCode, Rx, RxChannelConfig, RxChannelCreator, Tx, TxChannelConfig,
        TxChannelCreator,
    },
};

pub struct OneWire<'a> {
    rx: Channel<'a, Async, Rx>,
    tx: Channel<'a, Async, Tx>,
    input: InputSignal<'a>,
}

const MAX_EXCHANGE_PULSES: usize = 65;

impl<'a> OneWire<'a> {
    pub fn new<Txc: TxChannelCreator<'a, Async>, Rxc: RxChannelCreator<'a, Async>, P: Pin + 'a>(
        txcc: Txc,
        rxcc: Rxc,
        pin: P,
    ) -> Result<Self, Error> {
        let rx_config = RxChannelConfig::default()
            .with_clk_divider(80)
            .with_idle_threshold(1000)
            .with_filter_threshold(10)
            .with_carrier_modulation(false);
        let tx_config = TxChannelConfig::default()
            .with_clk_divider(80)
            .with_carrier_modulation(false);

        let mut pin: Flex = Flex::new(pin);

        pin.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
        pin.apply_output_config(
            &OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_drive_strength(DriveStrength::_40mA),
        );
        pin.set_input_enable(true);
        pin.set_output_enable(true);
        let (input, output) = pin.split();

        let tx = txcc
            .configure_tx(output.with_output_inverter(true), tx_config)
            .map_err(Error::SendError)?;
        let rx = rxcc
            .configure_rx(input.clone().with_input_inverter(true), rx_config)
            .map_err(Error::ReceiveError)?;

        Ok(OneWire { rx, tx, input })
    }
}

impl<'a> OneWire<'a> {
    pub async fn reset(&mut self) -> Result<bool, Error> {
        let data = [
            PulseCode::new(Level::Low, 60, Level::High, 600),
            PulseCode::new(Level::Low, 600, Level::Low, 0),
            PulseCode::end_marker(),
        ];
        let mut indata = [PulseCode::end_marker().into(); 10];

        let _res = self.send_and_receive(&mut indata, &data).await?;

        let sample0 = PulseCode::from(indata[0]);
        let sample1 = PulseCode::from(indata[1]);
        const PRESENCE_MIN_TICKS: u16 = 30; // 30 µs with 1 MHz tick
        const PRESENCE_MAX_TICKS: u16 = 500; // generous upper bound for noisy buses

        Ok(sample0.length1() > 0
            && sample0.length2() > 0
            && sample1.length1() >= PRESENCE_MIN_TICKS
            && sample1.length1() <= PRESENCE_MAX_TICKS)
    }

    pub async fn send_and_receive(
        &mut self,
        indata: &mut [u32],
        data: &[PulseCode],
    ) -> Result<(), Error> {
        let delay = [PulseCode::new(Level::Low, 30000, Level::Low, 0)]; // timeout delay for 30ms using the RMT tx peripheral.
        if self.input.level() == Level::Low {
            Err(Error::InputNotHigh)?;
        }
        // This relies on select polling in order to set up the rx & tx registers, which is not strictly documented behavior.
        let res = select(self.rx.receive(indata), async {
            let r = self.tx.transmit(data).await;
            let _ = self.tx.transmit(&delay).await;
            r
        })
        .await;

        match res {
            Either::First(Ok(_)) => Ok(()),
            Either::First(Err(r)) => Err(Error::ReceiveError(r)),
            Either::Second(Ok(())) => Err(Error::ReceiveTimedOut),
            Either::Second(Err(e)) => Err(Error::SendError(e)),
        }
    }

    const ZERO_BIT_LEN: u16 = 70;
    const ONE_BIT_LEN: u16 = 3;

    pub fn encode_bit(bit: bool) -> PulseCode {
        if bit {
            PulseCode::new(
                Level::High,
                Self::ONE_BIT_LEN,
                Level::Low,
                Self::ZERO_BIT_LEN,
            )
        } else {
            PulseCode::new(
                Level::High,
                Self::ZERO_BIT_LEN,
                Level::Low,
                Self::ONE_BIT_LEN,
            )
        }
    }

    pub fn decode_bit(code: PulseCode) -> bool {
        let len = code.length1();
        if len < 20 { true } else { false }
    }

    pub async fn exchange_byte(&mut self, byte: u8) -> Result<u8, Error> {
        let mut data = [PulseCode::end_marker(); 10];
        let mut indata = [PulseCode::end_marker().into(); 10];
        for n in 0..8 {
            data[n] = Self::encode_bit(0 != byte & 1 << n);
        }
        let _res = self.send_and_receive(&mut indata, &data).await?;
        let mut res: u8 = 0;
        for n in 0..8 {
            if Self::decode_bit(PulseCode::from(indata[n])) {
                res |= 1 << n;
            }
        }
        Ok(res)
    }

    pub async fn send_byte(&mut self, byte: u8) -> Result<(), Error> {
        let mut data = [PulseCode::end_marker(); 10];
        for n in 0..8 {
            data[n] = Self::encode_bit(0 != byte & 1 << n);
        }
        let _res = self.tx.transmit(&data).await?;
        Ok(())
    }

    pub async fn exchange_bits<const N: usize>(
        &mut self,
        bits: [bool; N],
    ) -> Result<[bool; N], Error> {
        assert!(N + 1 <= MAX_EXCHANGE_PULSES);

        let mut tx_buf = [PulseCode::end_marker(); MAX_EXCHANGE_PULSES];
        let mut rx_buf = [PulseCode::end_marker().into(); MAX_EXCHANGE_PULSES];

        for (idx, bit) in bits.into_iter().enumerate() {
            tx_buf[idx] = Self::encode_bit(bit);
        }
        tx_buf[N] = PulseCode::end_marker();

        self.send_and_receive(&mut rx_buf[..=N], &tx_buf[..=N])
            .await?;

        let mut res: [bool; N] = [false; N];
        for n in 0..N {
            res[n] = Self::decode_bit(PulseCode::from(rx_buf[n]));
        }
        Ok(res)
    }

    pub async fn send_u64(&mut self, val: u64) -> Result<(), Error> {
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
    fn from(e: esp_hal::rmt::Error) -> Error {
        Error::SendError(e)
    }
}

#[derive(PartialEq, Eq, Clone, Copy)]
pub struct Address(pub u64);

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
    pub async fn next<'d>(&mut self, ow: &mut OneWire<'d>) -> Result<Address, SearchError> {
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
