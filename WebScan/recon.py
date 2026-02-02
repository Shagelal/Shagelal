import socket
import requests
from urllib.parse import urlparse

def resolve_dns(target):
    print("\n[+] Resolving DNS & IP address")
    try:
        parsed = urlparse(target)
        hostname = parsed.netloc
        ip_address = socket.gethostbyname(hostname)

        print(f"  Hostname : {hostname}")
        print(f"  IP Addr  : {ip_address}")
    except socket.gaierror:
        print("  [!] DNS resolution failed")

def get_headers(target):
    print(f"\n[+] Fetching HTTP headers from {target}")
    try:
        response = requests.get(target, timeout=10)
        headers = response.headers

        for key, value in headers.items():
            print(f"  {key}: {value}")

        return headers, response
    except requests.exceptions.RequestException as e:
        print(f"[!] Error connecting to target: {e}")
        return None, None

def detect_tech_stack(headers, response):
    print("\n[+] Detecting technology stack (passive)")

    tech_found = set()
    server = headers.get("Server", "")
    powered = headers.get("X-Powered-By", "")

    if server:
        tech_found.add(f"Server: {server}")
    if powered:
        tech_found.add(f"Powered-By: {powered}")

    body = response.text.lower()
    tech_signatures = {
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

    for sig, tech in tech_signatures.items():
        if sig in body:
            tech_found.add(tech)

    if tech_found:
        for t in tech_found:
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

    http_url = "http://" + parsed.netloc + parsed.path
    try:
        r = requests.get(http_url, timeout=8, allow_redirects=False)
        if r.status_code in [301, 302] and "https://" in r.headers.get("Location", ""):
            print("  [✓] HTTP redirects to HTTPS")
        else:
            print("  [!] HTTP does NOT redirect to HTTPS")
    except requests.exceptions.RequestException:
        print("  [-] Could not test HTTP redirect")

    if "Strict-Transport-Security" in headers:
        print("  [✓] HSTS header present")
    else:
        print("  [!] HSTS header missing")

def check_robots_and_security_txt(target):
    print("\n[+] Checking robots.txt & security.txt")
    parsed = urlparse(target)
    base = parsed.scheme + "://" + parsed.netloc

    files = {
        "/robots.txt": "robots.txt",
        "/.well-known/security.txt": "security.txt"
    }

    for path, name in files.items():
        try:
            r = requests.get(base + path, timeout=8)
            if r.status_code == 200:
                print(f"  [✓] {name} found ({path})")
                if name == "robots.txt":
                    lines = r.text.splitlines()
                    hints = [l for l in lines if l.lower().startswith("disallow")]
                    if hints:
                        print("     Disallowed paths hinted:")
                        for h in hints[:5]:
                            print(f"       {h}")
                if name == "security.txt":
                    print("     Security contact policy present")
            elif r.status_code in [401, 403]:
                print(f"  [✓] {name} exists but restricted")
            else:
                print(f"  [-] {name} not found")
        except requests.exceptions.RequestException:
            print(f"  [-] Could not fetch {name}")
