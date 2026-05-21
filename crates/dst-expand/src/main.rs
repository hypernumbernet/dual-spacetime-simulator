//! CLI for symbolic basis / sandwich expansions.

use dst_expand::{
    BasisMonomial, Coefficient, expand_basis_product, expand_sandwich, mul_table_markdown,
};
use dst_expand::format_expanded;
use std::env;
use std::process::ExitCode;

fn usage() -> &'static str {
    "dst-expand — symbolic tetraquaternion expansion\n\n\
     Usage:\n\
       dst-expand table              Print 15×15 basis multiplication table (Markdown)\n\
       dst-expand mul <i> <j>        Expand product of basis i and j (0..14)\n\
       dst-expand sandwich <l> <m> <r>  Expand sandwich with unit coefficients\n\n\
     Examples:\n\
       dst-expand mul 4 5\n\
       dst-expand sandwich 14 0 14\n"
}

fn parse_basis_index(arg: &str) -> Result<usize, ExitCode> {
    arg.parse::<usize>()
        .ok()
        .filter(|&i| i < 15)
        .ok_or_else(|| {
            eprintln!("error: basis index must be 0..14, got {arg:?}");
            ExitCode::from(2)
        })
}

fn main() -> ExitCode {
    let mut args = env::args().skip(1);
    let Some(cmd) = args.next() else {
        print!("{}", usage());
        return ExitCode::SUCCESS;
    };

    match cmd.as_str() {
        "table" => {
            println!("{}", mul_table_markdown());
            ExitCode::SUCCESS
        }
        "mul" => {
            let Some(a) = args.next() else {
                eprintln!("error: missing left basis index");
                return ExitCode::from(2);
            };
            let Some(b) = args.next() else {
                eprintln!("error: missing right basis index");
                return ExitCode::from(2);
            };
            let left = match parse_basis_index(&a) {
                Ok(i) => i,
                Err(c) => return c,
            };
            let right = match parse_basis_index(&b) {
                Ok(i) => i,
                Err(c) => return c,
            };
            let exp = expand_basis_product(left, right);
            println!("{}", format_expanded(&exp));
            ExitCode::SUCCESS
        }
        "sandwich" => {
            let indices: Result<Vec<usize>, ExitCode> = args
                .take(3)
                .map(|a| parse_basis_index(&a))
                .collect();
            let [l, m, r] = match indices {
                Ok(v) if v.len() == 3 => [v[0], v[1], v[2]],
                Ok(_) => {
                    eprintln!("error: sandwich requires exactly 3 basis indices");
                    return ExitCode::from(2);
                }
                Err(c) => return c,
            };
            let exp = expand_sandwich(
                &BasisMonomial::basis(l),
                Coefficient::one(),
                &BasisMonomial::basis(m),
                Coefficient::one(),
                &BasisMonomial::basis(r),
                Coefficient::one(),
            );
            println!("{}", format_expanded(&exp));
            ExitCode::SUCCESS
        }
        "-h" | "--help" | "help" => {
            print!("{}", usage());
            ExitCode::SUCCESS
        }
        other => {
            eprintln!("error: unknown command {other:?}\n");
            print!("{}", usage());
            ExitCode::from(2)
        }
    }
}
