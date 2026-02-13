/// HORUS SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Display HORUS ASCII art header with version information
pub fn show_ascii_art() {
    let c_h = "\x1b[96m";
    let c_o = "\x1b[94m";
    let c_r = "\x1b[95m";
    let c_u = "\x1b[35m";
    let c_s = "\x1b[91m";
    let c_dim = "\x1b[2m";
    let c_reset = "\x1b[0m";

    let rows = [
        (
            "\u{2588}\u{2588}\u{2557}  \u{2588}\u{2588}\u{2557} ",
            "\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2557} ",
            "\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2557} ",
            "\u{2588}\u{2588}\u{2557}   \u{2588}\u{2588}\u{2557}",
            "\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2557}",
        ),
        (
            "\u{2588}\u{2588}\u{2551}  \u{2588}\u{2588}\u{2551}",
            "\u{2588}\u{2588}\u{2554}\u{2550}\u{2550}\u{2550}\u{2588}\u{2588}\u{2557}",
            "\u{2588}\u{2588}\u{2554}\u{2550}\u{2550}\u{2588}\u{2588}\u{2557}",
            "\u{2588}\u{2588}\u{2551}   \u{2588}\u{2588}\u{2551}",
            "\u{2588}\u{2588}\u{2554}\u{2550}\u{2550}\u{2550}\u{2550}\u{255d}",
        ),
        (
            "\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2551}",
            "\u{2588}\u{2588}\u{2551}   \u{2588}\u{2588}\u{2551}",
            "\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2554}\u{255d}",
            "\u{2588}\u{2588}\u{2551}   \u{2588}\u{2588}\u{2551}",
            "\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2557}",
        ),
        (
            "\u{2588}\u{2588}\u{2554}\u{2550}\u{2550}\u{2588}\u{2588}\u{2551}",
            "\u{2588}\u{2588}\u{2551}   \u{2588}\u{2588}\u{2551}",
            "\u{2588}\u{2588}\u{2554}\u{2550}\u{2550}\u{2588}\u{2588}\u{2557}",
            "\u{2588}\u{2588}\u{2551}   \u{2588}\u{2588}\u{2551}",
            "\u{255a}\u{2550}\u{2550}\u{2550}\u{2550}\u{2588}\u{2588}\u{2551}",
        ),
        (
            "\u{2588}\u{2588}\u{2551}  \u{2588}\u{2588}\u{2551}",
            "\u{255a}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2554}\u{255d}",
            "\u{2588}\u{2588}\u{2551}  \u{2588}\u{2588}\u{2551}",
            "\u{255a}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2554}\u{255d}",
            "\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2588}\u{2551}",
        ),
        (
            "\u{255a}\u{2550}\u{255d}  \u{255a}\u{2550}\u{255d} ",
            "\u{255a}\u{2550}\u{2550}\u{2550}\u{2550}\u{2550}\u{255d} ",
            "\u{255a}\u{2550}\u{255d}  \u{255a}\u{2550}\u{255d} ",
            "\u{255a}\u{2550}\u{2550}\u{2550}\u{2550}\u{2550}\u{255d} ",
            "\u{255a}\u{2550}\u{2550}\u{2550}\u{2550}\u{2550}\u{2550}\u{255d}",
        ),
    ];

    println!();
    for (h, o, r, u, s) in rows {
        println!(
            "    {}{}{}{}{}{}{}{}{}{}{}{}{}{}{}",
            c_h, h, c_reset,
            c_o, o, c_reset,
            c_r, r, c_reset,
            c_u, u, c_reset,
            c_s, s, c_reset,
        );
    }

    println!();
    println!("    {}        Holistic Operational Reality{}", c_o, c_reset);
    println!("    {}            for Unified Systems{}", c_r, c_reset);
    println!();
    println!("    {}\u{1F916} Mixed Reality Robot Management Platform{}", c_dim, c_reset);
    println!(
        "    {}\u{1F3D7}\u{FE0F} Developed at RICE Lab, University of Genoa{}",
        c_dim, c_reset
    );
    println!("    {}HORUS SDK v{}{}", c_dim, VERSION, c_reset);
    println!();

    println!("============================================================");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version_not_empty() {
        assert!(!VERSION.is_empty());
    }

    #[test]
    fn test_show_ascii_art() {
        // Just make sure it doesn't panic
        show_ascii_art();
    }
}
