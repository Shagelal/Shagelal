import argparse

from utils import banner, validate_url
from recon import (
    resolve_dns,
    get_headers,
    detect_tech_stack,
    check_tls_https,
    check_robots_and_security_txt
)
from owasp import (
    check_security_headers,
    check_cookies,
    check_reflected_input,
    check_exposed_paths
)
from report import generate_summary, get_findings


def main():
    banner()

    parser = argparse.ArgumentParser(
        description="WebScan - Bug Bounty Recon & Vulnerability Signal Tool"
    )
    parser.add_argument(
        "-u",
        "--url",
        required=True,
        help="Target URL (example: https://example.com)"
    )

    args = parser.parse_args()
    target = validate_url(args.url)

    print(f"\n[+] Target set to: {target}")

    # =========================
    # Module 1 – Recon
    # =========================
    resolve_dns(target)
    headers, response = get_headers(target)

    if not headers or not response:
        print("\n[✗] Recon failed – unable to fetch target")
        return

    detect_tech_stack(headers, response)
    check_tls_https(target, headers)
    check_robots_and_security_txt(target)

    # =========================
    # Module 2 – OWASP Signals
    # =========================
    check_security_headers(headers)
    check_cookies(response)
    check_reflected_input(target)
    check_exposed_paths(target)

    # =========================
    # Scan Summary
    # =========================
    summary = generate_summary()

    print("\n=========================")
    print("      Scan Summary      ")
    print("=========================")

    for level, count in summary.items():
        print(f"{level}: {count}")

    if summary["HIGH"] > 0:
        overall_risk = "HIGH"
    elif summary["MEDIUM"] > 0:
        overall_risk = "MEDIUM"
    else:
        overall_risk = "LOW"

    print(f"\nOverall Risk Level: {overall_risk}")

    # =========================
    # Manual Verification Guidance
    # =========================
    findings = get_findings()

    if findings:
        print("\n==============================")
        print(" Manual Verification Guidance ")
        print("==============================")

        for f in findings:
            print("\n--------------------------------")
            print(f"Category : {f['category']}")
            print(f"Title    : {f['title']}")
            print(f"Severity : {f['severity']}")
            print(f"Location : {f['location']}")

            print("\nHow to manually verify:")
            for step in f["manual_verification"]:
                print(f"  - {step}")
    else:
        print("\n[✓] No significant vulnerability signals detected")

    print("\n[✓] Scan completed successfully")


if __name__ == "__main__":
    main()
