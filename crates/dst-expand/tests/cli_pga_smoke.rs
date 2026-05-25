//! CLI contract tests for G(3,1,1) PGA subcommands (`pga-table`, `pga-mul`, etc.).

use dst_expand::{
    expand_pga_basis_product, expand_pga_expr, format_pga_expanded, pga_mul_table_markdown,
};
use std::process::Command;

const EXIT_OK: i32 = 0;
const EXIT_USAGE_ERROR: i32 = 2;

const USAGE_PGA_TABLE: &str = "dst-expand pga-table";
const USAGE_PGA_MUL: &str = "dst-expand pga-mul <i> <j>";
const USAGE_PGA_SANDWICH: &str = "dst-expand pga-sandwich <l> <m> <r>";
const USAGE_PGA_EXPR: &str = "dst-expand pga-expr <expression>";

struct CliOutput {
    stdout: String,
    stderr: String,
    code: i32,
}

fn dst_expand() -> Command {
    Command::new(env!("CARGO_BIN_EXE_dst-expand"))
}

fn run(args: &[&str]) -> CliOutput {
    let output = dst_expand()
        .args(args)
        .output()
        .unwrap_or_else(|e| panic!("failed to run dst-expand {args:?}: {e}"));
    CliOutput {
        stdout: String::from_utf8_lossy(&output.stdout).into_owned(),
        stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
        code: output
            .status
            .code()
            .expect("dst-expand should exit with a numeric code"),
    }
}

fn assert_ok(out: &CliOutput, context: &str) {
    assert_eq!(
        out.code, EXIT_OK,
        "{context}: expected exit {EXIT_OK}, got {} (stderr={:?})",
        out.code, out.stderr
    );
}

fn assert_usage_error(out: &CliOutput, context: &str) {
    assert_eq!(
        out.code, EXIT_USAGE_ERROR,
        "{context}: expected exit {EXIT_USAGE_ERROR}, got {} (stderr={:?})",
        out.code, out.stderr
    );
    assert!(
        out.stderr.starts_with("error:"),
        "{context}: stderr should start with `error:`, got {:?}",
        out.stderr
    );
}

fn assert_stdout_trimmed(out: &CliOutput, expected: &str, context: &str) {
    assert_eq!(
        out.stdout.trim(),
        expected,
        "{context}: stdout mismatch (stderr={:?})",
        out.stderr
    );
}

fn assert_pga_usage_text(text: &str, context: &str) {
    assert!(text.contains(USAGE_PGA_TABLE), "{context}: missing pga-table");
    assert!(text.contains(USAGE_PGA_MUL), "{context}: missing pga-mul");
    assert!(
        text.contains(USAGE_PGA_SANDWICH),
        "{context}: missing pga-sandwich"
    );
    assert!(text.contains(USAGE_PGA_EXPR), "{context}: missing pga-expr");
}

// --- usage ---

#[test]
fn help_includes_pga_commands() {
    let out = run(&["help"]);
    assert_ok(&out, "help");
    assert_pga_usage_text(&out.stdout, "help");
}

// --- pga-table ---

#[test]
fn pga_table_prints_markdown_multiplication_table() {
    let out = run(&["pga-table"]);
    assert_ok(&out, "pga-table");
    let lines: Vec<&str> = out.stdout.lines().collect();
    assert!(
        lines.len() >= 34,
        "table should have header + separator + 32 rows, got {} lines",
        lines.len()
    );
    assert!(lines[0].starts_with("| × |"), "header: {:?}", lines[0]);
    assert!(lines[1].starts_with("|---|"), "separator: {:?}", lines[1]);
    for (i, row) in lines.iter().skip(2).take(32).enumerate() {
        let cells: Vec<&str> = row.split('|').filter(|s| !s.is_empty()).collect();
        assert_eq!(
            cells.len(),
            33,
            "data row {i} should have 33 cells (label + 32 columns)"
        );
    }
}

#[test]
fn pga_table_e0_squared_minus_one() {
    let out = run(&["pga-table"]);
    assert_ok(&out, "pga-table");
    let row = out
        .stdout
        .lines()
        .find(|line| line.contains("**e0**"))
        .expect("row for basis e0");
    assert!(
        row.contains(" -1 |") || row.ends_with(" -1 |"),
        "e0×e0 cell should be -1, row={row:?}"
    );
}

#[test]
fn pga_table_e4_squared_zero() {
    let out = run(&["pga-table"]);
    assert_ok(&out, "pga-table");
    let row = out
        .stdout
        .lines()
        .find(|line| line.contains("**e4**"))
        .expect("row for basis e4");
    assert!(
        row.contains(" 0 |"),
        "e4×e4 cell should be 0, row={row:?}"
    );
}

#[test]
fn pga_table_rejects_extra_arguments() {
    let out = run(&["pga-table", "extra"]);
    assert_usage_error(&out, "pga-table extra arg");
    assert!(
        out.stderr.contains("pga-table takes exactly 0 argument"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn pga_table_matches_library() {
    let out = run(&["pga-table"]);
    assert_ok(&out, "pga-table");
    assert_eq!(out.stdout, format!("{}\n", pga_mul_table_markdown()));
}

// --- pga-mul ---

#[test]
fn pga_mul_e0_e0_to_minus_one() {
    let out = run(&["pga-mul", "1", "1"]);
    assert_ok(&out, "pga-mul 1 1");
    assert_stdout_trimmed(&out, "-1", "e0*e0");
}

#[test]
fn pga_mul_e4_e4_to_zero() {
    let out = run(&["pga-mul", "16", "16"]);
    assert_ok(&out, "pga-mul 16 16");
    assert_stdout_trimmed(&out, "0", "e4*e4");
}

#[test]
fn pga_mul_rejects_out_of_range_index() {
    let out = run(&["pga-mul", "32", "0"]);
    assert_usage_error(&out, "pga-mul range");
    assert!(
        out.stderr.contains("basis index must be 0..31"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn pga_mul_rejects_missing_indices() {
    let out = run(&["pga-mul"]);
    assert_usage_error(&out, "pga-mul no args");
    assert!(
        out.stderr.contains("missing left basis index"),
        "stderr={:?}",
        out.stderr
    );
    let out = run(&["pga-mul", "1"]);
    assert_usage_error(&out, "pga-mul one arg");
    assert!(
        out.stderr.contains("missing right basis index"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn pga_mul_rejects_extra_arguments() {
    let out = run(&["pga-mul", "1", "2", "3"]);
    assert_usage_error(&out, "pga-mul extra");
    assert!(
        out.stderr.contains("pga-mul takes exactly 2 argument"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn pga_mul_cross_check_library() {
    let left = 1;
    let right = 2;
    let out = run(&["pga-mul", &left.to_string(), &right.to_string()]);
    assert_ok(&out, "pga-mul cross-check");
    let expected = format_pga_expanded(&expand_pga_basis_product(left, right));
    assert_stdout_trimmed(&out, &expected, "pga-mul cross-check");
}

// --- pga-sandwich ---

#[test]
fn pga_sandwich_requires_three_indices() {
    let out = run(&["pga-sandwich", "1", "2"]);
    assert_usage_error(&out, "pga-sandwich two args");
    assert!(
        out.stderr.contains("pga-sandwich requires exactly 3 basis indices"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn pga_sandwich_rejects_four_arguments() {
    let out = run(&["pga-sandwich", "1", "2", "3", "4"]);
    assert_usage_error(&out, "pga-sandwich four args");
    assert!(
        out.stderr.contains("pga-sandwich takes exactly 3 argument"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn pga_sandwich_rejects_invalid_index() {
    let out = run(&["pga-sandwich", "1", "2", "99"]);
    assert_usage_error(&out, "pga-sandwich invalid");
    assert!(
        out.stderr.contains("basis index must be 0..31"),
        "stderr={:?}",
        out.stderr
    );
}

// --- pga-expr ---

#[test]
fn pga_expr_e0_squared() {
    let out = run(&["pga-expr", "(e0)(e0)"]);
    assert_ok(&out, "pga-expr e0^2");
    assert_stdout_trimmed(&out, "-1", "e0^2");
}

#[test]
fn pga_expr_e4_squared_is_zero() {
    let out = run(&["pga-expr", "e4e4"]);
    assert_ok(&out, "pga-expr e4^2");
    assert_stdout_trimmed(&out, "0", "e4^2");
}

#[test]
fn pga_expr_anticommute_sum_is_zero() {
    let out = run(&["pga-expr", "e0e1 + e1e0"]);
    assert_ok(&out, "pga-expr anticommute");
    assert_stdout_trimmed(&out, "0", "anticommute");
}

#[test]
fn pga_expr_coeff_sum() {
    let out = run(&["pga-expr", "ae0+be1"]);
    assert_ok(&out, "pga-expr coeff sum");
    assert_stdout_trimmed(&out, "a[e0] + b[e1]", "coeff sum");
}

#[test]
fn pga_expr_parse_error() {
    let out = run(&["pga-expr", "(e0"]);
    assert_usage_error(&out, "pga-expr parse error");
    assert!(
        out.stderr.contains("at offset"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn pga_expr_missing_expression() {
    let out = run(&["pga-expr"]);
    assert_usage_error(&out, "pga-expr no args");
    assert!(
        out.stderr.contains("missing expression"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn pga_expr_cross_check_library() {
    let expr = "ae0+be1";
    let out = run(&["pga-expr", expr]);
    assert_ok(&out, "pga-expr cross-check");
    let expected = format_pga_expanded(&expand_pga_expr(expr).unwrap());
    assert_stdout_trimmed(&out, &expected, "pga-expr cross-check");
}
