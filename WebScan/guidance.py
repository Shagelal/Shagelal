GUIDANCE = {
    "IDOR": [
        "Login as a low-privileged user",
        "Modify the object ID (numeric or UUID)",
        "Compare response with original request",
        "Confirm authorization is enforced server-side"
    ],

    "XSS": [
        "Identify reflection context (HTML, attribute, JS)",
        "Test harmless HTML tags manually",
        "Check if output encoding is applied",
        "Avoid automated payload injection"
    ],

    "OPEN_REDIRECT": [
        "Supply an external URL manually",
        "Observe redirect behavior",
        "Check for allowlist validation",
        "Confirm redirect target control"
    ],

    "CORS": [
        "Inspect CORS headers carefully",
        "Test cross-origin request manually",
        "Check if credentials are allowed",
        "Verify sensitive data exposure"
    ],

    "EXPOSED_PATH": [
        "Access endpoint in authenticated vs unauthenticated state",
        "Verify role-based access control",
        "Check if sensitive functionality is exposed"
    ],

    "SECURITY_HEADERS": [
        "Review missing headers",
        "Assess impact on browser security",
        "Confirm with application context"
    ]
}
