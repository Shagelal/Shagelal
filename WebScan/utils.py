from urllib.parse import urlparse

def banner():
    print("=" * 60)
    print(" WebScan - Automated Recon & OWASP Basic Checker ")
    print(" Authorized testing only ")
    print("=" * 60)

def validate_url(target):
    parsed = urlparse(target)
    if not parsed.scheme:
        return "http://" + target
    return target