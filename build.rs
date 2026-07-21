fn main() {
    println!("cargo:rustc-link-search=.");
    println!("cargo:rustc-link-arg-bins=-Tcoffee-linkall.x");
}
