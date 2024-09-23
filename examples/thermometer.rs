#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_println::println;
use embassy_time::{Duration, Timer};
use esp_hal::{gpio::Io, rmt::{*, asynch::*}, peripherals::Peripherals, prelude::*, timer::timg::TimerGroup, clock::ClockControl, system::SystemControl};
use esp_hal_rmt_onewire::*;

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let clocks = ClockControl::boot_defaults(SystemControl::new(peripherals.SYSTEM).clock_control).freeze();
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    esp_hal_embassy::init(&clocks, timer_group0.timer0);
    let rmt = Rmt::new_async(peripherals.RMT, 80_u32.MHz(), &clocks).unwrap();
    let mut ow = OneWire::new(rmt.channel0, rmt.channel2, io.pins.gpio6);
    // let mut ow = OneWire::new(rmt.channel0, rmt.channel2, io.pins.gpio6);// Flex::new(io.pins.gpio7));
    
    loop {
        println!("Resetting the bus");
        ow.reset().await;

        println!("Broadcasting a measure temperature command to all attached sensors");
        for a in [0xCC, 0x44] {
            ow.send_byte(a).await;
        }

        println!("Scanning the bus to retrieve the measured temperatures");
        let samples = search(&mut ow).await;

        println!("Waiting for 10 seconds");
        Timer::after(Duration::from_secs(10)).await;
    }
}

// Temperature in C
#[derive(Ord, PartialOrd, PartialEq, Eq, Debug)]
pub struct Temperature(pub fixed::types::I12F4);

const CTOF_FACT: fixed::types::I12F4 = fixed::types::I12F4::lit("1.8");
const CTOF_OFF: fixed::types::I12F4 = fixed::types::I12F4::lit("32");

impl core::fmt::Display for Temperature {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> Result<(), core::fmt::Error> {
        write!(f, "{}°F ({}°C)", self.0 * CTOF_FACT + CTOF_OFF, self.0)?;
        Ok(())
    }
}

pub async fn search<R: RxChannelAsync, T: TxChannelAsync>(ow: &mut OneWire<R, T>) -> () {
    let mut search = Search::new();
    loop {
        match search.next(ow).await {
            Ok(address) => {
                println!("Reading device {:?}", address);
                ow.reset().await;
                ow.send_byte(0x55).await;
                ow.send_address(address).await;
                ow.send_byte(0xBE).await;
                let temp_low = ow.exchange_byte(0xFF).await;
                let temp_high = ow.exchange_byte(0xFF).await;
                let temp = fixed::types::I12F4::from_le_bytes([temp_low, temp_high]);
                println!("Temp is: {temp}");
            }
            Err(_) => {
                println!("End of search");
                return ();
            }
        }
    }
}
