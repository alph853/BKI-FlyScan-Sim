from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, RedirectResponse
from pydantic import BaseModel
import os
import uvicorn
from network_manager import NetworkManager

app = FastAPI()
nm = NetworkManager()

class WiFiCredentials(BaseModel):
    ssid: str
    password: str = ""

@app.get("/")
async def root():
    # Redirect to config page
    return RedirectResponse(url="/config")

@app.get("/config")
async def config_page():
    with open("/app/frontend/index.html", "r") as f:
        return HTMLResponse(content=f.read())

@app.get("/api/networks")
async def get_networks():
    """Get list of available WiFi networks"""
    try:
        networks = nm.scan_networks()
        return {"networks": networks}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/saved-networks")
async def get_saved_networks():
    """Get list of saved network SSIDs"""
    try:
        saved = nm.get_saved_networks()
        return {"saved_networks": saved}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/connect")
async def connect_wifi(credentials: WiFiCredentials):
    """Connect to selected WiFi network"""
    try:
        # Check if network is saved
        saved_networks = nm.get_saved_networks()
        
        if credentials.ssid in saved_networks and not credentials.password:
            # Use saved credentials
            result = nm.connect_to_saved(credentials.ssid)
        else:
            # Connect with new credentials
            result = nm.connect_to_network(credentials.ssid, credentials.password)
        
        if result["success"]:
            # Disable hotspot after successful connection
            nm.disable_hotspot()
            
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# Serve static files
app.mount("/static", StaticFiles(directory="/app/frontend"), name="static")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=80)