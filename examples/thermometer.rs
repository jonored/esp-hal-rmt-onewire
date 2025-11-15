#![no_std]
#![no_main]

#[cfg(not(any(feature = "example-esp32c3", feature = "example-esp32c6")))]
compile_error!(
    "Enable either the `example-esp32c3` or `example-esp32c6` feature to build this example."
);

#[cfg(all(feature = "example-esp32c3", feature = "example-esp32c6"))]
compile_error!("Only one example target feature can be enabled at a time.");

use embassy_executor::{Executor, task};
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, delay::Delay, rmt::*, time::Rate};
use esp_hal_rmt_onewire::{OneWire, Search};
use esp_println::println;
use static_cell::StaticCell;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[cfg(feature = "example-deps")]
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    #[cfg(feature = "example-esp32c3")]
    let onewire_pin = peripherals.GPIO6;

    #[cfg(feature = "example-esp32c6")]
    let onewire_pin = peripherals.GPIO18;

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80_u32))
        .unwrap()
        .into_async();
    let ow = OneWire::new(rmt.channel0, rmt.channel2, onewire_pin).unwrap();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(move |spawner| {
        spawner
            .spawn(thermometer(ow))
            .expect("failed to start thermometer task");
    })
}

#[task]
async fn thermometer(mut ow: OneWire<'static>) -> ! {
    let delay = Delay::new();

    loop {
        println!("Resetting the bus");
        ow.reset().await.unwrap();

        println!("Broadcasting a measure temperature command to all attached sensors");
        for a in [0xCC, 0x44] {
            ow.send_byte(a).await.unwrap();
        }

        println!("Scanning the bus to retrieve the measured temperatures");
        search(&mut ow).await;

        println!("Waiting for 10 seconds");
        delay.delay_millis(10_000);
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

pub async fn search<'a>(ow: &mut OneWire<'a>) -> () {
    let mut search = Search::new();
    loop {
        match search.next(ow).await {
            Ok(address) => {
                println!("Reading device {:?}", address);
                ow.reset().await.unwrap();
                ow.send_byte(0x55).await.unwrap();
                ow.send_address(address).await.unwrap();
                ow.send_byte(0xBE).await.unwrap();
                let temp_low = ow
                    .exchange_byte(0xFF)
                    .await
                    .expect("failed to get low byte of temperature");
                let temp_high = ow
                    .exchange_byte(0xFF)
                    .await
                    .expect("failed to get high byte of temperature");
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
