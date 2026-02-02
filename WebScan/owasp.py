import requests
from urllib.parse import urlparse, urlencode
import html

from report import add_finding
from guidance import GUIDANCE


def check_security_headers(headers):
    print("\n[+] Checking security headers (OWASP: Security Misconfiguration)")

    required_headers = {
        "Content-Security-Policy": "Helps prevent XSS attacks",
        "X-Frame-Options": "Protects against clickjacking",
        "Strict-Transport-Security": "Enforces HTTPS",
        "X-Content-Type-Options": "Prevents MIME sniffing"
    }

    missing = []

    for header, desc in required_headers.items():
        if header in headers:
            print(f"  [✓] {header} present")
        else:
            print(f"  [!] {header} MISSING – {desc}")
            missing.append(header)

    if missing:
        add_finding(
            category="Security Misconfiguration",
            title="Missing security headers",
            severity="MEDIUM",
            location="HTTP response headers",
            evidence=missing,
            manual_steps=GUIDANCE["SECURITY_HEADERS"]
        )


def check_cookies(response):
    print("\n[+] Checking cookies (OWASP: Identification & Authentication Failures)")

    cookies = response.cookies
    if not cookies:
        print("  [✓] No cookies detected")
        return

    set_cookie_header = response.headers.get("Set-Cookie", "").lower()
    insecure = []

    for cookie in cookies:
        print(f"\n  Cookie Name: {cookie.name}")
        issues = []

        if not cookie.secure:
            print("   [!] Secure flag MISSING")
            issues.append("Secure flag missing")
        else:
            print("   [✓] Secure flag set")

        if "httponly" not in set_cookie_header:
            print("   [!] HttpOnly flag MISSING")
            issues.append("HttpOnly flag missing")
        else:
            print("   [✓] HttpOnly flag set")

        if issues:
            insecure.append({
                "cookie": cookie.name,
                "issues": issues
            })

    if insecure:
        add_finding(
            category="Authentication Failures",
            title="Insecure cookie configuration",
            severity="MEDIUM",
            location="Set-Cookie headers",
            evidence=insecure,
            manual_steps=GUIDANCE["IDOR"]
        )


def check_reflected_input(target):
    print("\n[+] Checking reflected input (XSS signal)")

    markers = ["webscan123"]
    parameters = ["q", "search", "query", "id"]

    parsed = urlparse(target)
    base_url = parsed.scheme + "://" + parsed.netloc + parsed.path

    findings = []

    for param in parameters:
        for marker in markers:
            payload = marker
            encoded_payload = html.escape(marker)
            test_url = base_url + "?" + urlencode({param: payload})

            try:
                r = requests.get(test_url, timeout=10)
                body = r.text.lower()

                if payload.lower() in body or encoded_payload.lower() in body:
                    index = body.find(payload.lower())
                    snippet = body[max(0, index - 40): index + 40]

                    print(f"  [!] Reflection detected in parameter: {param}")

                    findings.append({
                        "parameter": param,
                        "marker": marker,
                        "snippet": snippet
                    })

            except requests.exceptions.RequestException:
                continue

    if findings:
        add_finding(
            category="XSS (Signal)",
            title="Reflected input detected",
            severity="MEDIUM",
            location=base_url,
            evidence=findings,
            manual_steps=GUIDANCE["XSS"]
        )
    else:
        print("  [✓] No reflected input detected")


def check_exposed_paths(target):
    print("\n[+] Checking for exposed admin / debug paths")

    paths = [
        "/admin", "/login", "/admin/login", "/dashboard",
        "/debug", "/.env", "/phpinfo.php",
        "/server-status", "/wp-admin"
    ]

    parsed = urlparse(target)
    base = parsed.scheme + "://" + parsed.netloc
    exposed = []

    for path in paths:
        try:
            r = requests.get(base + path, timeout=8, allow_redirects=False)
            status = r.status_code

            if status in [200, 301, 302]:
                print(f"  [!] {path} accessible (HTTP {status})")
                exposed.append({
                    "path": path,
                    "status": status
                })
            elif status in [401, 403]:
                print(f"  [✓] {path} restricted (HTTP {status})")
            else:
                print(f"  [-] {path} not found (HTTP {status})")

        except requests.exceptions.RequestException:
            continue

    if exposed:
        add_finding(
            category="Broken Access Control",
            title="Exposed sensitive endpoints",
            severity="MEDIUM",
            location=base,
            evidence=exposed,
            manual_steps=GUIDANCE["EXPOSED_PATH"]
        )
