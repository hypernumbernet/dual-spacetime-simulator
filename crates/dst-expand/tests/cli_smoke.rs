use std::process::Command;

#[test]
fn mul_subcommand_runs() {
    let exe = env!("CARGO_BIN_EXE_dst-expand");
    let out = Command::new(exe)
        .args(["mul", "0", "0"])
        .output()
        .expect("run dst-expand");
    assert!(out.status.success(), "stderr: {}", String::from_utf8_lossy(&out.stderr));
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(!stdout.trim().is_empty());
}

#[test]
fn expr_subcommand_expands_product() {
    let exe = env!("CARGO_BIN_EXE_dst-expand");
    let out = Command::new(exe)
        .args(["expr", "(j)(j)"])
        .output()
        .expect("run dst-expand expr");
    assert!(out.status.success(), "stderr: {}", String::from_utf8_lossy(&out.stderr));
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert_eq!(stdout.trim(), "-1");
}
