#[cfg(target_os = "macos")]
fn main() {
    // Allow unresolved Python symbols to be looked up at runtime by the
    // interpreter when building the extension module on macOS.
    println!("cargo:rustc-cdylib-link-arg=-undefined");
    println!("cargo:rustc-cdylib-link-arg=dynamic_lookup");
}

#[cfg(not(target_os = "macos"))]
fn main() {}
