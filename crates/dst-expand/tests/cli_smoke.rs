//! CLI contract tests for `dst-expand`.
//!
//! These tests encode the Phase 0 specification in
//! [`docs/dst-expand-本格数式処理仕様.md`](../../docs/dst-expand-本格数式処理仕様.md).
//! They assert desired CLI behaviour rather than incidental implementation details.

use std::process::Command;

const EXIT_OK: i32 = 0;
const EXIT_USAGE_ERROR: i32 = 2;

const USAGE_MARKER: &str = "dst-expand — symbolic tetraquaternion expansion";
const USAGE_TABLE: &str = "dst-expand table";
const USAGE_MUL: &str = "dst-expand mul <i> <j>";
const USAGE_SANDWICH: &str = "dst-expand sandwich <l> <m> <r>";
const USAGE_EXPR: &str = "dst-expand expr <expression>";

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

fn assert_usage_text(text: &str, context: &str) {
    assert!(
        text.contains(USAGE_MARKER),
        "{context}: missing banner"
    );
    assert!(text.contains(USAGE_TABLE), "{context}: missing table usage");
    assert!(text.contains(USAGE_MUL), "{context}: missing mul usage");
    assert!(
        text.contains(USAGE_SANDWICH),
        "{context}: missing sandwich usage"
    );
    assert!(text.contains(USAGE_EXPR), "{context}: missing expr usage");
}

// --- usage / help ---

#[test]
fn no_args_prints_usage_to_stdout_and_exits_ok() {
    let out = run(&[]);
    assert_ok(&out, "no args");
    assert_usage_text(&out.stdout, "no args");
    assert!(
        out.stderr.is_empty(),
        "no args should not write stderr, got {:?}",
        out.stderr
    );
}

#[test]
fn help_flag_prints_usage_to_stdout_and_exits_ok() {
    for flag in ["help", "-h", "--help"] {
        let out = run(&[flag]);
        assert_ok(&out, flag);
        assert_usage_text(&out.stdout, flag);
        assert!(out.stderr.is_empty(), "{flag} should not write stderr");
    }
}

#[test]
fn unknown_command_prints_error_and_usage() {
    let out = run(&["nosuch"]);
    assert_usage_error(&out, "unknown command");
    assert!(
        out.stderr.contains("unknown command"),
        "stderr={:?}",
        out.stderr
    );
    assert_usage_text(&out.stdout, "unknown command");
}

// --- table ---

#[test]
fn table_prints_markdown_multiplication_table() {
    let out = run(&["table"]);
    assert_ok(&out, "table");
    let lines: Vec<&str> = out.stdout.lines().collect();
    assert!(
        lines.len() >= 17,
        "table should have header + separator + 15 rows, got {} lines",
        lines.len()
    );
    assert!(
        lines[0].starts_with("| × |"),
        "first line should be header, got {:?}",
        lines[0]
    );
    assert!(
        lines[1].starts_with("|---|"),
        "second line should be separator, got {:?}",
        lines[1]
    );
    for (i, row) in lines.iter().skip(2).take(15).enumerate() {
        assert!(
            row.starts_with('|'),
            "data row {i} should start with '|', got {row:?}"
        );
        let cells: Vec<&str> = row.split('|').filter(|s| !s.is_empty()).collect();
        assert_eq!(
            cells.len(),
            16,
            "data row {i} should have 16 cells (label + 15 columns)"
        );
    }
}

#[test]
fn table_rejects_extra_arguments() {
    let out = run(&["table", "extra"]);
    assert_usage_error(&out, "table extra arg");
    assert!(
        out.stderr.contains("table takes exactly 0 argument"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn table_scalar_cell_is_signed_one() {
    let out = run(&["table"]);
    assert_ok(&out, "table");
    // j × j = -1 (indices 0,0)
    let row = out
        .stdout
        .lines()
        .find(|line| line.contains("**j**"))
        .expect("row for basis j");
    assert!(
        row.contains(" -1 |") || row.ends_with(" -1 |"),
        "j×j cell should be -1, row={row:?}"
    );
}

// --- mul ---

#[test]
fn mul_expands_basis_product_j_j_to_minus_one() {
    let out = run(&["mul", "0", "0"]);
    assert_ok(&out, "mul 0 0");
    assert_stdout_trimmed(&out, "-1", "mul j*j");
}

#[test]
fn mul_expands_basis_product_i_j_to_k() {
    let out = run(&["mul", "14", "0"]);
    assert_ok(&out, "mul 14 0");
    assert_stdout_trimmed(&out, "[k]", "mul i*j");
}

#[test]
fn mul_example_from_usage_i_i_i_j() {
    let out = run(&["mul", "4", "5"]);
    assert_ok(&out, "mul 4 5");
    assert!(
        !out.stdout.trim().is_empty(),
        "mul 4 5 should produce output"
    );
}

#[test]
fn mul_rejects_missing_left_index() {
    let out = run(&["mul"]);
    assert_usage_error(&out, "mul no args");
    assert!(
        out.stderr.contains("missing left basis index"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn mul_rejects_missing_right_index() {
    let out = run(&["mul", "0"]);
    assert_usage_error(&out, "mul one arg");
    assert!(
        out.stderr.contains("missing right basis index"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn mul_rejects_extra_argument() {
    let out = run(&["mul", "0", "0", "0"]);
    assert_usage_error(&out, "mul extra arg");
    assert!(
        out.stderr.contains("mul takes exactly 2 argument"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn mul_rejects_non_numeric_index() {
    let out = run(&["mul", "abc", "0"]);
    assert_usage_error(&out, "mul non-numeric");
    assert!(
        out.stderr.contains("basis index must be 0..14"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn mul_rejects_out_of_range_index() {
    for bad in ["15", "99"] {
        let out = run(&["mul", bad, "0"]);
        assert_usage_error(&out, "mul out of range");
        assert!(
            out.stderr.contains("basis index must be 0..14"),
            "bad={bad:?} stderr={:?}",
            out.stderr
        );
    }
}

// --- sandwich ---

#[test]
fn sandwich_j_j_j_is_minus_j() {
    let out = run(&["sandwich", "0", "0", "0"]);
    assert_ok(&out, "sandwich j j j");
    assert_stdout_trimmed(&out, "-[j]", "sandwich j j j");
}

#[test]
fn sandwich_example_from_usage_i_j_i() {
    let out = run(&["sandwich", "14", "0", "14"]);
    assert_ok(&out, "sandwich 14 0 14");
    assert_stdout_trimmed(&out, "[j]", "sandwich i j i");
}

#[test]
fn sandwich_rejects_fewer_than_three_indices() {
    let cases: [&[&str]; 3] = [
        &["sandwich"],
        &["sandwich", "0"],
        &["sandwich", "0", "0"],
    ];
    for args in cases {
        let out = run(&args);
        assert_usage_error(&out, "sandwich too few");
        assert!(
            out.stderr.contains("sandwich requires exactly 3 basis indices"),
            "args={args:?} stderr={:?}",
            out.stderr
        );
    }
}

#[test]
fn sandwich_rejects_extra_argument() {
    let out = run(&["sandwich", "0", "0", "0", "0"]);
    assert_usage_error(&out, "sandwich extra arg");
    assert!(
        out.stderr.contains("sandwich takes exactly 3 argument"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn sandwich_rejects_invalid_index() {
    let out = run(&["sandwich", "0", "0", "15"]);
    assert_usage_error(&out, "sandwich bad index");
    assert!(
        out.stderr.contains("basis index must be 0..14"),
        "stderr={:?}",
        out.stderr
    );
}

// --- expr ---

#[test]
fn expr_j_squared_is_minus_one() {
    let out = run(&["expr", "(j)(j)"]);
    assert_ok(&out, "expr (j)(j)");
    assert_stdout_trimmed(&out, "-1", "expr j*j");
}

#[test]
fn expr_supports_coefficient_sum() {
    let out = run(&["expr", "ai+bkI"]);
    assert_ok(&out, "expr sum");
    let text = out.stdout.trim();
    assert!(text.contains('a'), "stdout={text:?}");
    assert!(text.contains('b'), "stdout={text:?}");
    assert!(text.contains("[i]"), "stdout={text:?}");
    assert!(text.contains("[kI]"), "stdout={text:?}");
}

#[test]
fn expr_supports_explicit_star_and_juxtaposition() {
    let star = run(&["expr", "a*i"]);
    let juxta = run(&["expr", "ai"]);
    assert_ok(&star, "expr a*i");
    assert_ok(&juxta, "expr ai");
    assert_eq!(
        star.stdout.trim(),
        juxta.stdout.trim(),
        "explicit * and juxtaposition should match"
    );
}

#[test]
fn expr_supports_whitespace() {
    let out = run(&["expr", " ai + bkI "]);
    assert_ok(&out, "expr whitespace");
    let compact = run(&["expr", "ai+bkI"]);
    assert_eq!(
        out.stdout.trim(),
        compact.stdout.trim(),
        "whitespace should not change result"
    );
}

#[test]
fn expr_expands_parenthesized_product() {
    let out = run(&["expr", "(ai+bkI)(cj+dkK)"]);
    assert_ok(&out, "expr product");
    assert!(
        !out.stdout.trim().is_empty(),
        "expanded product should not be empty"
    );
}

#[test]
fn expr_rejects_missing_expression() {
    let out = run(&["expr"]);
    assert_usage_error(&out, "expr no args");
    assert!(
        out.stderr.contains("missing expression"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn expr_rejects_multiple_expression_arguments() {
    let out = run(&["expr", "j", "j"]);
    assert_usage_error(&out, "expr two args");
    assert!(
        out.stderr.contains("expr takes exactly 1 argument"),
        "stderr={:?}",
        out.stderr
    );
}

#[test]
fn expr_reports_parse_error_on_stderr() {
    let out = run(&["expr", "(ai"]);
    assert_usage_error(&out, "expr parse error");
    assert!(
        out.stderr.contains("error:") && out.stderr.contains("expected ')'"),
        "stderr={:?}",
        out.stderr
    );
    assert!(
        out.stdout.is_empty(),
        "parse error should not write stdout, got {:?}",
        out.stdout
    );
}

#[test]
fn expr_reports_trailing_input_error() {
    let out = run(&["expr", "j)"]);
    assert_usage_error(&out, "expr trailing");
    assert!(
        out.stderr.contains("unexpected trailing input"),
        "stderr={:?}",
        out.stderr
    );
}

// --- cross-check: CLI output matches library formatting ---

#[test]
fn mul_cli_matches_library_format_expanded() {
    use dst_expand::{expand_basis_product, format_expanded};

    for (left, right) in [(0, 0), (14, 0), (4, 5), (10, 14)] {
        let expected = format_expanded(&expand_basis_product(left, right));
        let out = run(&["mul", &left.to_string(), &right.to_string()]);
        assert_ok(&out, "mul cross-check");
        assert_stdout_trimmed(
            &out,
            &expected,
            &format!("mul {left} {right} vs library"),
        );
    }
}

#[test]
fn expr_cli_matches_library_format_expanded() {
    use dst_expand::{expand_expr, format_expanded};

    for expression in ["(j)(j)", "ai+bkI", "a*i", "(ai+bkI)(cj+dkK)"] {
        let expected = format_expanded(&expand_expr(expression).expect("library expand"));
        let out = run(&["expr", expression]);
        assert_ok(&out, "expr cross-check");
        assert_stdout_trimmed(
            &out,
            &expected,
            &format!("expr {expression:?} vs library"),
        );
    }
}
