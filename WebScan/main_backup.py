#!/usr/bin/env python3

import argparse
import requests
import socket
from urllib.parse import urlparse, urlencode
import html

# =====================================================
# Banner & Utils
# =====================================================

def banner():
    print("=" * 60)
    print(" WebScan - Bug Bounty Recon & Vulnerability Signal Tool ")
    print(" Authorized testing only ")
    print("=" * 60)

def validate_url(target):
    parsed = urlparse(target)
    if not parsed.scheme:
        return "http://" + target
    return target

# =====================================================
# Central Findings Engine
# =====================================================

FINDINGS = []

def add_finding(category, title, severity, location, evidence, manual_steps):
    FINDINGS.append({
        "category": category,
        "title": title,
        "severity": severity,
        "location": location,
        "evidence": evidence,
        "manual_verification": manual_steps
    })

def generate_summary():
    summary = {"LOW": 0, "MEDIUM": 0, "HIGH": 0}
    for f in FINDINGS:
        summary[f["severity"]] += 1
    return summary

# =====================================================
# Manual Guidance Engine
# =====================================================

GUIDANCE = {
    "XSS": [
        "Identify reflection context (HTML, attribute, JS)",
        "Test harmless HTML manually",
        "Check output encoding",
        "Confirm sanitization logic"
    ],
    "IDOR": [
        "Login as a low-privileged user",
        "Identify object ID parameter",
        "Change ID manually",
        "Compare access control behavior"
    ],
    "EXPOSED_PATH": [
        "Access endpoint unauthenticated",
        "Test with different roles",
        "Verify authorization enforcement"
    ],
    "SECURITY_HEADERS": [
        "Review missing headers",
        "Understand browser-side impact",
        "Confirm required mitigations"
    ],
    "COOKIES": [
        "Check Secure and HttpOnly flags",
        "Assess session handling",
        "Verify protection of sensitive actions"
    ],
    "ROBOTS": [
        "Review disallowed paths",
        "Check if sensitive endpoints are listed",
        "Manually verify access control on those paths"
    ]
}

# =====================================================
# Module 1 – Recon
# =====================================================

def resolve_dns(target):
    print("\n[+] Resolving DNS & IP address")
    try:
        hostname = urlparse(target).netloc
        ip_address = socket.gethostbyname(hostname)
        print(f"  Hostname : {hostname}")
        print(f"  IP Addr  : {ip_address}")
    except socket.gaierror:
        print("  [!] DNS resolution failed")

def get_headers(target):
    print(f"\n[+] Fetching HTTP headers from {target}")
    try:
        r = requests.get(target, timeout=10)
        for k, v in r.headers.items():
            print(f"  {k}: {v}")
        return r.headers, r
    except requests.exceptions.RequestException as e:
        print(f"[!] Error connecting to target: {e}")
        return None, None

def detect_tech_stack(headers, response):
    print("\n[+] Detecting technology stack (passive)")
    found = set()

    if headers.get("Server"):
        found.add(f"Server: {headers['Server']}")
    if headers.get("X-Powered-By"):
        found.add(f"Powered-By: {headers['X-Powered-By']}")

    body = response.text.lower()
    signatures = {
        "wordpress": "WordPress",
        "wp-content": "WordPress",
        "drupal": "Drupal",
        "joomla": "Joomla",
        "laravel": "Laravel",
        "django": "Django",
        "asp.net": "ASP.NET",
        "php": "PHP",
        "react": "React",
        "angular": "Angular",
        "vue": "Vue.js",
        "jquery": "jQuery"
    }

    for sig, tech in signatures.items():
        if sig in body:
            found.add(tech)

    if found:
        for t in found:
            print(f"  [✓] {t}")
    else:
        print("  [-] No obvious technologies detected")

def check_tls_https(target, headers):
    print("\n[+] Checking TLS / HTTPS configuration")
    parsed = urlparse(target)

    if parsed.scheme == "https":
        print("  [✓] HTTPS is in use")
    else:
        print("  [!] HTTPS is NOT in use")

    if "Strict-Transport-Security" in headers:
        print("  [✓] HSTS header present")
    else:
        print("  [!] HSTS header missing")

def check_robots_and_security_txt(target):
    print("\n[+] Checking robots.txt & security.txt")

    base = urlparse(target).scheme + "://" + urlparse(target).netloc

    # -------- robots.txt --------
    try:
        r = requests.get(base + "/robots.txt", timeout=8)
        if r.status_code == 200:
            print("  [✓] robots.txt found")
            lines = r.text.splitlines()
            disallowed = [l for l in lines if l.lower().startswith("disallow")]

            if disallowed:
                print("     Disallowed paths:")
                for d in disallowed[:10]:
                    print(f"       {d}")

                add_finding(
                    "Information Disclosure",
                    "Sensitive paths disclosed via robots.txt",
                    "LOW",
                    "robots.txt",
                    disallowed,
                    GUIDANCE["ROBOTS"]
                )
        else:
            print("  [-] robots.txt not found")
    except requests.exceptions.RequestException:
        print("  [-] Could not fetch robots.txt")

    # -------- security.txt --------
    try:
        r = requests.get(base + "/.well-known/security.txt", timeout=8)
        if r.status_code == 200:
            print("  [✓] security.txt found (security contact present)")
        else:
            print("  [-] security.txt not found")
    except requests.exceptions.RequestException:
        print("  [-] Could not fetch security.txt")

# =====================================================
# Module 2 – OWASP Signal Checks (NO PAYLOADS)
# =====================================================

def check_security_headers(headers):
    print("\n[+] Checking security headers")

    required = [
        "Content-Security-Policy",
        "X-Frame-Options",
        "Strict-Transport-Security",
        "X-Content-Type-Options"
    ]

    missing = [h for h in required if h not in headers]

    for h in required:
        print(f"  [{'✓' if h in headers else '!'}] {h}")

    if missing:
        add_finding(
            "Security Misconfiguration",
            "Missing security headers",
            "MEDIUM",
            "HTTP headers",
            missing,
            GUIDANCE["SECURITY_HEADERS"]
        )

def check_cookies(response):
    print("\n[+] Checking cookies")

    if not response.cookies:
        print("  [✓] No cookies detected")
        return

    issues = []
    sc = response.headers.get("Set-Cookie", "").lower()

    for c in response.cookies:
        ci = []
        if not c.secure:
            ci.append("Secure flag missing")
        if "httponly" not in sc:
            ci.append("HttpOnly flag missing")
        if ci:
            print(f"  [!] Cookie {c.name}: {ci}")
            issues.append({c.name: ci})

    if issues:
        add_finding(
            "Authentication Failures",
            "Insecure cookie configuration",
            "MEDIUM",
            "Cookies",
            issues,
            GUIDANCE["COOKIES"]
        )

def check_reflected_input(target):
    print("\n[+] Checking reflected input (XSS signal)")

    params = ["q", "search", "query", "id"]
    marker = "webscan123"
    parsed = urlparse(target)
    base = parsed.scheme + "://" + parsed.netloc + parsed.path
    hits = []

    for p in params:
        try:
            r = requests.get(base + "?" + urlencode({p: marker}), timeout=10)
            if marker.lower() in r.text.lower():
                print(f"  [!] Reflection detected in parameter: {p}")
                hits.append(p)
        except requests.exceptions.RequestException:
            continue

    if hits:
        add_finding(
            "XSS (Signal)",
            "Reflected input detected",
            "MEDIUM",
            base,
            hits,
            GUIDANCE["XSS"]
        )

def check_exposed_paths(target):
    print("\n[+] Checking exposed admin / debug paths")

    paths = [
        "/admin", "/login", "/dashboard",
        "/debug", "/.env", "/phpinfo.php", "/wp-admin"
    ]

    base = urlparse(target).scheme + "://" + urlparse(target).netloc
    exposed = []

    for p in paths:
        try:
            r = requests.get(base + p, timeout=8, allow_redirects=False)
            if r.status_code in [200, 301, 302]:
                print(f"  [!] {p} accessible (HTTP {r.status_code})")
                exposed.append(p)
        except requests.exceptions.RequestException:
            continue

    if exposed:
        add_finding(
            "Broken Access Control",
            "Exposed sensitive paths",
            "MEDIUM",
            base,
            exposed,
            GUIDANCE["EXPOSED_PATH"]
        )

# =====================================================
# Main
# =====================================================

def main():
    banner()

    parser = argparse.ArgumentParser(description="WebScan - Bug Bounty Recon Tool")
    parser.add_argument("-u", "--url", required=True)
    args = parser.parse_args()

    target = validate_url(args.url)
    print(f"\n[+] Target set to: {target}")

    resolve_dns(target)
    headers, response = get_headers(target)
    if not headers:
        print("\n[✗] Scan failed")
        return

    detect_tech_stack(headers, response)
    check_tls_https(target, headers)
    check_robots_and_security_txt(target)

    check_security_headers(headers)
    check_cookies(response)
    check_reflected_input(target)
    check_exposed_paths(target)

    summary = generate_summary()

    print("\n=========================")
    print("      Scan Summary      ")
    print("=========================")
    for k, v in summary.items():
        print(f"{k}: {v}")

    print("\n===== Manual Verification Guidance =====")
    for f in FINDINGS:
        print("\n--------------------------------------")
        print(f"Category : {f['category']}")
        print(f"Title    : {f['title']}")
        print(f"Severity : {f['severity']}")
        print(f"Location : {f['location']}")
        print("How to manually verify:")
        for step in f["manual_verification"]:
            print(f"  - {step}")

    print("\n[✓] Scan completed successfully")

if __name__ == "__main__":
    main()
