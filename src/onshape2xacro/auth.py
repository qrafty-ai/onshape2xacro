"""
Cross-platform credential storage for Onshape API keys using keyring.
"""

import os
import keyring

SERVICE_NAME = "onshape2xacro"
ACCESS_KEY_USERNAME = "access_key"
SECRET_KEY_USERNAME = "secret_key"


def store_credentials(access_key: str, secret_key: str) -> None:
    """Store Onshape API credentials in the system keyring."""
    keyring.set_password(SERVICE_NAME, ACCESS_KEY_USERNAME, access_key)
    keyring.set_password(SERVICE_NAME, SECRET_KEY_USERNAME, secret_key)


def get_credentials() -> tuple[str | None, str | None]:
    """
    Retrieve Onshape API credentials.

    Priority:
    1. Environment variables (ONSHAPE_ACCESS_KEY, ONSHAPE_SECRET_KEY)
    2. System keyring

    Returns:
        Tuple of (access_key, secret_key), either may be None if not found.
    """
    # Check environment variables first
    access_key = os.environ.get("ONSHAPE_ACCESS_KEY")
    secret_key = os.environ.get("ONSHAPE_SECRET_KEY")

    if access_key and secret_key:
        return access_key, secret_key

    # Fall back to keyring
    access_key = keyring.get_password(SERVICE_NAME, ACCESS_KEY_USERNAME)
    secret_key = keyring.get_password(SERVICE_NAME, SECRET_KEY_USERNAME)

    return access_key, secret_key


def delete_credentials() -> None:
    """Remove stored credentials from the system keyring."""
    try:
        keyring.delete_password(SERVICE_NAME, ACCESS_KEY_USERNAME)
    except keyring.errors.PasswordDeleteError:
        pass  # Key didn't exist

    try:
        keyring.delete_password(SERVICE_NAME, SECRET_KEY_USERNAME)
    except keyring.errors.PasswordDeleteError:
        pass  # Key didn't exist


def has_stored_credentials() -> bool:
    """Check if credentials are stored in the keyring."""
    access_key = keyring.get_password(SERVICE_NAME, ACCESS_KEY_USERNAME)
    secret_key = keyring.get_password(SERVICE_NAME, SECRET_KEY_USERNAME)
    return bool(access_key and secret_key)
