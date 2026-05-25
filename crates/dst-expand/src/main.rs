//! CLI for symbolic basis / sandwich expansions.
//!
//! # Contract (Phase 0)
//!
//! - Success: exit code `0`, result on stdout.
//! - Usage: no args, `help`, `-h`, `--help` → usage on stdout, exit `0`.
//! - User error: message on stderr prefixed with `error:`, exit code `2`.
//! - `mul` / `sandwich` / `expr` accept a fixed number of arguments only.
//! - `table` accepts no arguments.

use dst_expand::format_expanded;
use dst_expand::format_pga_expanded;
use dst_expand::{
    BasisMonomial, Coefficient, expand_basis_product, expand_expr, expand_sandwich,
    mul_table_markdown,
};
use dst_expand::{
    PgaBasisMonomial, expand_pga_basis_product, expand_pga_expr, expand_pga_sandwich,
    pga_mul_table_markdown,
};
use dst_math::pga::PGA_DIM;
use std::env;
use std::process::ExitCode;

const EXIT_SUCCESS: u8 = 0;
const EXIT_USAGE_ERROR: u8 = 2;

fn usage() -> &'static str {
    "dst-expand — symbolic biquaternion expansion\n\n\
     Usage (biquaternion / Cl(3,1)):\n\
       dst-expand table              Print 15×15 basis multiplication table (Markdown)\n\
       dst-expand mul <i> <j>        Expand product of basis i and j (0..14)\n\
       dst-expand sandwich <l> <m> <r>  Expand sandwich with unit coefficients\n\
       dst-expand expr <expression>  Expand a coefficient-bearing expression\n\n\
     Usage (G(3,1,1) PGA):\n\
       dst-expand pga-table              Print 32×32 PGA basis multiplication table\n\
       dst-expand pga-mul <i> <j>        Expand product of basis i and j (0..31)\n\
       dst-expand pga-sandwich <l> <m> <r>  Expand sandwich with unit coefficients\n\
       dst-expand pga-expr <expression>  Expand with e0..e4 generators\n\n\
     Examples:\n\
       dst-expand mul 4 5\n\
       dst-expand sandwich 14 0 14\n\
       dst-expand expr \"(ai+bkI)(cj+dkK)\"\n\
       dst-expand pga-mul 1 1\n\
       dst-expand pga-expr \"(e0)(e0)\"\n"
}

fn user_error(message: impl AsRef<str>) -> ExitCode {
    eprintln!("error: {}", message.as_ref());
    ExitCode::from(EXIT_USAGE_ERROR)
}

fn parse_basis_index(arg: &str) -> Result<usize, ExitCode> {
    arg.parse::<usize>()
        .ok()
        .filter(|&i| i < 15)
        .ok_or_else(|| user_error(format!("basis index must be 0..14, got {arg:?}")))
}

fn parse_pga_basis_index(arg: &str) -> Result<usize, ExitCode> {
    arg.parse::<usize>()
        .ok()
        .filter(|&i| i < PGA_DIM)
        .ok_or_else(|| user_error(format!("basis index must be 0..31, got {arg:?}")))
}

/// Collects exactly `count` trailing CLI arguments or returns a user error.
fn take_exact_args<I>(args: I, count: usize, command: &str) -> Result<Vec<String>, ExitCode>
where
    I: Iterator<Item = String>,
{
    let collected: Vec<String> = args.collect();
    match collected.len().cmp(&count) {
        std::cmp::Ordering::Equal => Ok(collected),
        std::cmp::Ordering::Less => {
            if command == "mul" || command == "pga-mul" {
                match collected.len() {
                    0 => Err(user_error("missing left basis index")),
                    1 => Err(user_error("missing right basis index")),
                    _ => Err(user_error(format!(
                        "{command} requires exactly {count} basis indices"
                    ))),
                }
            } else if command == "sandwich" || command == "pga-sandwich" {
                Err(user_error(format!(
                    "{command} requires exactly {count} basis indices"
                )))
            } else if command == "expr" || command == "pga-expr" {
                Err(user_error("missing expression"))
            } else {
                Err(user_error(format!(
                    "{command} requires exactly {count} argument(s)"
                )))
            }
        }
        std::cmp::Ordering::Greater => Err(user_error(format!(
            "{command} takes exactly {count} argument(s)"
        ))),
    }
}

fn parse_basis_indices(args: &[String]) -> Result<Vec<usize>, ExitCode> {
    args.iter()
        .map(|a| parse_basis_index(a))
        .collect::<Result<Vec<_>, _>>()
}

fn cmd_table<I>(args: I) -> ExitCode
where
    I: Iterator<Item = String>,
{
    let trailing = take_exact_args(args, 0, "table");
    match trailing {
        Ok(_) => {
            println!("{}", mul_table_markdown());
            ExitCode::from(EXIT_SUCCESS)
        }
        Err(code) => code,
    }
}

fn cmd_mul<I>(args: I) -> ExitCode
where
    I: Iterator<Item = String>,
{
    let indices = match take_exact_args(args, 2, "mul") {
        Ok(v) => v,
        Err(code) => return code,
    };
    let parsed = match parse_basis_indices(&indices) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let [left, right] = [parsed[0], parsed[1]];
    let exp = expand_basis_product(left, right);
    println!("{}", format_expanded(&exp));
    ExitCode::from(EXIT_SUCCESS)
}

fn cmd_sandwich<I>(args: I) -> ExitCode
where
    I: Iterator<Item = String>,
{
    let indices = match take_exact_args(args, 3, "sandwich") {
        Ok(v) => v,
        Err(code) => return code,
    };
    let parsed = match parse_basis_indices(&indices) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let [l, m, r] = [parsed[0], parsed[1], parsed[2]];
    let exp = expand_sandwich(
        &BasisMonomial::basis(l),
        Coefficient::one(),
        &BasisMonomial::basis(m),
        Coefficient::one(),
        &BasisMonomial::basis(r),
        Coefficient::one(),
    );
    println!("{}", format_expanded(&exp));
    ExitCode::from(EXIT_SUCCESS)
}

fn parse_pga_basis_indices(args: &[String]) -> Result<Vec<usize>, ExitCode> {
    args.iter()
        .map(|a| parse_pga_basis_index(a))
        .collect::<Result<Vec<_>, _>>()
}

fn cmd_pga_table<I>(args: I) -> ExitCode
where
    I: Iterator<Item = String>,
{
    match take_exact_args(args, 0, "pga-table") {
        Ok(_) => {
            println!("{}", pga_mul_table_markdown());
            ExitCode::from(EXIT_SUCCESS)
        }
        Err(code) => code,
    }
}

fn cmd_pga_mul<I>(args: I) -> ExitCode
where
    I: Iterator<Item = String>,
{
    let indices = match take_exact_args(args, 2, "pga-mul") {
        Ok(v) => v,
        Err(code) => return code,
    };
    let parsed = match parse_pga_basis_indices(&indices) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let [left, right] = [parsed[0], parsed[1]];
    let exp = expand_pga_basis_product(left, right);
    println!("{}", format_pga_expanded(&exp));
    ExitCode::from(EXIT_SUCCESS)
}

fn cmd_pga_sandwich<I>(args: I) -> ExitCode
where
    I: Iterator<Item = String>,
{
    let indices = match take_exact_args(args, 3, "pga-sandwich") {
        Ok(v) => v,
        Err(code) => return code,
    };
    let parsed = match parse_pga_basis_indices(&indices) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let [l, m, r] = [parsed[0], parsed[1], parsed[2]];
    let exp = expand_pga_sandwich(
        &PgaBasisMonomial::basis(l),
        Coefficient::one(),
        &PgaBasisMonomial::basis(m),
        Coefficient::one(),
        &PgaBasisMonomial::basis(r),
        Coefficient::one(),
    );
    println!("{}", format_pga_expanded(&exp));
    ExitCode::from(EXIT_SUCCESS)
}

fn cmd_pga_expr<I>(args: I) -> ExitCode
where
    I: Iterator<Item = String>,
{
    let expressions = match take_exact_args(args, 1, "pga-expr") {
        Ok(v) => v,
        Err(code) => return code,
    };
    match expand_pga_expr(&expressions[0]) {
        Ok(exp) => {
            println!("{}", format_pga_expanded(&exp));
            ExitCode::from(EXIT_SUCCESS)
        }
        Err(e) => user_error(e.to_string()),
    }
}

fn cmd_expr<I>(args: I) -> ExitCode
where
    I: Iterator<Item = String>,
{
    let expressions = match take_exact_args(args, 1, "expr") {
        Ok(v) => v,
        Err(code) => return code,
    };
    match expand_expr(&expressions[0]) {
        Ok(exp) => {
            println!("{}", format_expanded(&exp));
            ExitCode::from(EXIT_SUCCESS)
        }
        Err(e) => user_error(e.to_string()),
    }
}

fn main() -> ExitCode {
    let mut args = env::args().skip(1);
    let Some(cmd) = args.next() else {
        print!("{}", usage());
        return ExitCode::from(EXIT_SUCCESS);
    };

    match cmd.as_str() {
        "table" => cmd_table(args),
        "mul" => cmd_mul(args),
        "sandwich" => cmd_sandwich(args),
        "expr" => cmd_expr(args),
        "pga-table" => cmd_pga_table(args),
        "pga-mul" => cmd_pga_mul(args),
        "pga-sandwich" => cmd_pga_sandwich(args),
        "pga-expr" => cmd_pga_expr(args),
        "-h" | "--help" | "help" => {
            print!("{}", usage());
            ExitCode::from(EXIT_SUCCESS)
        }
        other => {
            eprintln!("error: unknown command {other:?}\n");
            print!("{}", usage());
            ExitCode::from(EXIT_USAGE_ERROR)
        }
    }
}
