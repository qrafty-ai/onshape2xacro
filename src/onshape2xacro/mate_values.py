import json
import logging
from pathlib import Path
from typing import Dict, Any

from onshape_robotics_toolkit.connect import HTTP

logger = logging.getLogger(__name__)


def fetch_mate_values(
    client, did: str, wvm: str, wvmid: str, eid: str
) -> Dict[str, Any]:
    """
    Fetch mate values from Onshape API.
    Returns a dict mapping mateId (featureId) to its values.

    Args:
        client: onshape_robotics_toolkit.Client instance
        did: Document ID
        wvm: Workspace/Version/Microversion type ('w', 'v', 'm')
        wvmid: Workspace/Version/Microversion ID
        eid: Element ID (Assembly)
    """
    # Endpoint: /api/assemblies/d/{did}/{wvm}/{wvmid}/e/{eid}/matevalues
    path = f"/api/assemblies/d/{did}/{wvm}/{wvmid}/e/{eid}/matevalues"

    try:
        response = client.request(HTTP.GET, path)
        if response.status_code != 200:
            logger.warning(
                f"Failed to fetch mate values: {response.status_code} {response.text}"
            )
            return {}

        data = response.json()
        mate_values = {}
        for entry in data.get("mateValues", []):
            # entry has featureId, mateName, rotationX, rotationY, rotationZ, translationX, ...
            mate_id = entry.get("featureId")
            if mate_id:
                mate_values[mate_id] = entry

        logger.info(f"Fetched values for {len(mate_values)} mates")
        return mate_values
    except Exception as e:
        logger.warning(f"Error fetching mate values: {e}")
        return {}


def save_mate_values(path: Path, values: Dict[str, Any]):
    """Save mate values to a JSON file."""
    with open(path, "w") as f:
        json.dump(values, f, indent=2)


def load_mate_values(path: Path) -> Dict[str, Any]:
    """Load mate values from a JSON file."""
    if not path.exists():
        return {}
    with open(path, "r") as f:
        return json.load(f)
