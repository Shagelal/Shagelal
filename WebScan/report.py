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

def get_findings():
    return FINDINGS
