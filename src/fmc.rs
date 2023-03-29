//! HAL for Flexible memory controller (FMC)
//!
//! FMC support is implemented via the
//! [stm32-fmc](https://github.com/stm32-rs/stm32-fmc) crate.
//!
//! ## SDRAM
//!
//! An external SDRAM can be instantiated by calling the [sdram](FmcExt::sdram)
//! extension method. To avoid the pin checks, you can use
//! [sdram_unchecked](FmcExt::sdram_unchecked) instead.
//!
//! ```
//! use stm32f4xx_hal::prelude::*;
//!
//! let sdram_pins = ...; // Tuple, see stm32-fmc docs for pin ordering
//! let sdram_chip = ...; // See stm32-fmc docs
//!
//! let mut sdram = dp.FMC.sdram(
//!     sdram_pins,
//!     sdram_chip,
//!     &ccdr.clocks,
//! );
//! ```
//!
//! `sdram` usage is described
//! [here](https://github.com/stm32-rs/stm32-fmc#usage).

// From stm32_fmc
use stm32_fmc::FmcPeripheral;
use stm32_fmc::{AddressPinSet, PinsSdram, Sdram, SdramChip, SdramPinSet, SdramTargetBank};

use crate::pac;
use crate::rcc::Clocks;
use crate::time::Hertz;

use crate::gpio::gpioc::PC0;
use crate::gpio::gpiod::{PD0, PD1, PD10, PD14, PD15, PD8, PD9};
use crate::gpio::gpioe::{PE0, PE1, PE10, PE11, PE12, PE13, PE14, PE15, PE7, PE8, PE9};
use crate::gpio::gpiof::{PF0, PF1, PF11, PF12, PF13, PF14, PF15, PF2, PF3, PF4, PF5};
use crate::gpio::gpiog::{PG0, PG1, PG15, PG4, PG5, PG8};
use crate::gpio::gpioh::{PH10, PH11, PH12, PH13, PH14, PH15, PH2, PH3, PH8, PH9};
use crate::gpio::gpioi::{PI0, PI1, PI10, PI2, PI3, PI4, PI5, PI6, PI7, PI9};
use crate::gpio::{Alternate, PB5, PB6, PG2};

/// Storage type for Flexible Memory Controller and its clocks
///
/// AHB access to the FMC peripheral must be enabled
pub struct FMC {
    fmc: pac::FSMC,
    fmc_ker_ck: Option<Hertz>,
}

/// Extension trait for FMC controller
pub trait FmcExt: Sized {
    fn fmc(self, clocks: &Clocks) -> FMC;

    /// A new SDRAM memory via the Flexible Memory Controler
    fn sdram<
        BANK: SdramPinSet,
        ADDR: AddressPinSet,
        PINS: PinsSdram<BANK, ADDR>,
        CHIP: SdramChip,
    >(
        self,
        pins: PINS,
        chip: CHIP,
        clocks: &Clocks,
    ) -> Sdram<FMC, CHIP> {
        let fmc = self.fmc(clocks);
        Sdram::new(fmc, pins, chip)
    }

    /// A new SDRAM memory via the Felxible Memory Controller
    fn sdram_unchecked<CHIP: SdramChip, BANK: Into<SdramTargetBank>>(
        self,
        bank: BANK,
        chip: CHIP,
        clocks: &Clocks,
    ) -> Sdram<FMC, CHIP> {
        let fmc = self.fmc(clocks);
        Sdram::new_unchecked(fmc, bank, chip)
    }
}
impl FmcExt for pac::FSMC {
    // New FMC instance
    fn fmc(self, clocks: &Clocks) -> FMC {
        const RCC: *const pac::rcc::RegisterBlock = pac::RCC::ptr();
        unsafe {
            (*RCC).ahb3enr.modify(|_, w| w.fmcen().enabled());
        }
        FMC {
            fmc: self,
            fmc_ker_ck: Some(clocks.hclk()),
        }
    }
}
unsafe impl FmcPeripheral for FMC {
    const REGISTERS: *const () = pac::FSMC::ptr() as *const ();

    fn enable(&mut self) {
        // Already enabled as part of the contract for creating FMC
    }

    fn memory_controller_enable(&mut self) {
        // The FMCEN bit of the FMC_BCR2..4 registers is don't
        // care. It is only enabled through the FMC_BCR1 register.
        self.fmc.bcr1.modify(|_, w| w.faccen().set_bit());
    }
    fn source_clock_hz(&self) -> u32 {
        // Check that it runs
        self.fmc_ker_ck
            .expect("FC kernel clock is not running!")
            .raw()
    }
}

macro_rules! pins {
    (FMC: $($pin:ident: [$($inst:ty),*])+) => {
        $(
            $(
                impl stm32_fmc::$pin for $inst {}
            )*
        )+
    }
}

pins! {
    FMC:
        A0: [ PF0<Alternate<12>> ]
        A1: [ PF1<Alternate<12>> ]
        A2: [ PF2<Alternate<12>> ]
        A3: [ PF3<Alternate<12>> ]
        A4: [ PF4<Alternate<12>> ]
        A5: [ PF5<Alternate<12>> ]
        A6: [ PF12<Alternate<12>> ]
        A7: [ PF13<Alternate<12>> ]
        A8: [ PF14<Alternate<12>> ]
        A9: [ PF15<Alternate<12>> ]
        A10: [ PG0<Alternate<12>> ]
        A11: [ PG1<Alternate<12>> ]
        A12: [ PG2<Alternate<12>> ]
        BA0: [ PG4<Alternate<12>> ]
        BA1: [ PG5<Alternate<12>> ]

        D0: [ PD14<Alternate<12>> ]
        D1: [ PD15<Alternate<12>> ]
        D2: [ PD0<Alternate<12>> ]
        D3: [ PD1<Alternate<12>> ]
        D4: [ PE7<Alternate<12>> ]
        D5: [ PE8<Alternate<12>> ]
        D6: [ PE9<Alternate<12>> ]
        D7: [ PE10<Alternate<12>> ]
        D8: [ PE11<Alternate<12>> ]
        D9: [ PE12<Alternate<12>> ]
        D10: [ PE13<Alternate<12>> ]
        D11: [ PE14<Alternate<12>> ]
        D12: [ PE15<Alternate<12>> ]
        D13: [ PD8<Alternate<12>> ]
        D14: [ PD9<Alternate<12>> ]
        D15: [ PD10<Alternate<12>> ]
        D16: [ PH8<Alternate<12>> ]
        D17: [ PH9<Alternate<12>> ]
        D18: [ PH10<Alternate<12>> ]
        D19: [ PH11<Alternate<12>> ]
        D20: [ PH12<Alternate<12>> ]
        D21: [ PH13<Alternate<12>> ]
        D22: [ PH14<Alternate<12>> ]
        D23: [ PH15<Alternate<12>> ]
        D24: [ PI0<Alternate<12>> ]
        D25: [ PI1<Alternate<12>> ]
        D26: [ PI2<Alternate<12>> ]
        D27: [ PI3<Alternate<12>> ]
        D28: [ PI6<Alternate<12>> ]
        D29: [ PI7<Alternate<12>> ]
        D30: [ PI9<Alternate<12>> ]
        D31: [ PI10<Alternate<12>> ]


        NBL0: [ PE0<Alternate<12>> ]
        NBL1: [ PE1<Alternate<12>> ]
        NBL2: [ PI4<Alternate<12>> ]
        NBL3: [ PI5<Alternate<12>> ]

        // SDRAM
        SDCKE0:[ PH2<Alternate<12>> ]
        SDCKE1:[ PB5<Alternate<12>> ]
        SDCLK: [ PG8<Alternate<12>> ]
        SDNCAS:[ PG15<Alternate<12>> ]
        SDNE0: [ PH3<Alternate<12>> ]
        SDNE1: [ PB6<Alternate<12>> ]
        SDNRAS:[ PF11<Alternate<12>> ]
        SDNWE: [ PC0<Alternate<12>> ]
}
