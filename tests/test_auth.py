import os
import keyring
from unittest.mock import patch
from onshape2xacro.auth import (
    store_credentials,
    get_credentials,
    delete_credentials,
    has_stored_credentials,
    SERVICE_NAME,
    ACCESS_KEY_USERNAME,
    SECRET_KEY_USERNAME,
)


def test_store_credentials():
    with patch("keyring.set_password") as mock_set:
        store_credentials("fake_access", "fake_secret")
        assert mock_set.call_count == 2
        mock_set.assert_any_call(SERVICE_NAME, ACCESS_KEY_USERNAME, "fake_access")
        mock_set.assert_any_call(SERVICE_NAME, SECRET_KEY_USERNAME, "fake_secret")


def test_get_credentials_env_priority():
    with patch.dict(
        os.environ,
        {"ONSHAPE_ACCESS_KEY": "env_access", "ONSHAPE_SECRET_KEY": "env_secret"},
    ):
        with patch("keyring.get_password") as mock_get:
            access, secret = get_credentials()
            assert access == "env_access"
            assert secret == "env_secret"
            mock_get.assert_not_called()


def test_get_credentials_keyring_fallback():
    with patch.dict(os.environ, {}, clear=True):
        with patch("keyring.get_password") as mock_get:
            mock_get.side_effect = (
                lambda svc, user: "key_access"
                if user == ACCESS_KEY_USERNAME
                else "key_secret"
            )
            access, secret = get_credentials()
            assert access == "key_access"
            assert secret == "key_secret"
            assert mock_get.call_count == 2


def test_get_credentials_none():
    with patch.dict(os.environ, {}, clear=True):
        with patch("keyring.get_password", return_value=None):
            access, secret = get_credentials()
            assert access is None
            assert secret is None


def test_delete_credentials():
    with patch("keyring.delete_password") as mock_del:
        delete_credentials()
        assert mock_del.call_count == 2
        mock_del.assert_any_call(SERVICE_NAME, ACCESS_KEY_USERNAME)
        mock_del.assert_any_call(SERVICE_NAME, SECRET_KEY_USERNAME)


def test_delete_credentials_handles_error():
    with patch("keyring.delete_password") as mock_del:
        mock_del.side_effect = keyring.errors.PasswordDeleteError("Not found")
        delete_credentials()
        assert mock_del.call_count == 2


def test_has_stored_credentials():
    with patch("keyring.get_password") as mock_get:
        mock_get.return_value = "something"
        assert has_stored_credentials() is True

        mock_get.return_value = None
        assert has_stored_credentials() is False
