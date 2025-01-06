/// build for standalone non-rust dependencies such as boot.S

use std::process::Command;

const CC: &str = "arm-none-eabi-as";
const ARGS: &[&str] = &["-mcpu=cortex-m4", "-mthumb", "-Wall"];

fn main() {
    println!("cargo:rerun-if-changed=boot.S");
    println!("Running build.rs...");

    let ret = Command::new(CC)
        .args(ARGS)
        .arg("boot.S")
        .arg("-o")
        .arg("target/thumbv7em-none-eabi/debug/boot.o")
        .status()
        .unwrap();

    if !ret.success() {
        println!("ERROR: Assembly of boot.S failed");
    }
}
