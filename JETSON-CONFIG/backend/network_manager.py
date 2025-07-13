import subprocess
import time
import re
import os

class NetworkManager:
    def __init__(self):
        self.interface = "wlan0"
        
    def scan_networks(self):
        """Scan for available WiFi networks"""
        try:
            # Use nmcli to scan
            cmd = ["nmcli", "-t", "-f", "SSID,SIGNAL,FREQ,SECURITY", "dev", "wifi", "list"]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            networks = []
            seen_ssids = set()
            
            for line in result.stdout.strip().split('\n'):
                if line:
                    parts = line.split(':')
                    if len(parts) >= 4 and parts[0] and parts[0] not in seen_ssids:
                        ssid = parts[0]
                        signal = int(parts[1]) if parts[1] else 0
                        freq = float(parts[2]) if parts[2] else 0
                        security = parts[3] if len(parts) > 3 else ""
                        
                        # Determine band
                        band = "5GHz" if freq > 5000 else "2.4GHz"
                        
                        networks.append({
                            "ssid": ssid,
                            "signal": signal,
                            "band": band,
                            "security": security,
                            "secured": bool(security and security != "--")
                        })
                        seen_ssids.add(ssid)
            
            # Sort by signal strength
            networks.sort(key=lambda x: x["signal"], reverse=True)
            return networks
            
        except Exception as e:
            print(f"Error scanning networks: {e}")
            return []
    
    def get_saved_networks(self):
        """Get list of saved network SSIDs"""
        try:
            cmd = ["nmcli", "-t", "-f", "NAME", "connection", "show"]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            saved = []
            for line in result.stdout.strip().split('\n'):
                if line and not line.startswith("UAV-JETSON-CONFIG"):
                    saved.append(line)
            
            return saved
            
        except Exception as e:
            print(f"Error getting saved networks: {e}")
            return []
    
    def connect_to_network(self, ssid, password):
        """Connect to a WiFi network with credentials"""
        try:
            # Delete existing connection if exists
            subprocess.run(["nmcli", "connection", "delete", ssid], 
                         capture_output=True)
            
            # Create new connection
            if password:
                cmd = [
                    "nmcli", "dev", "wifi", "connect", ssid,
                    "password", password, "ifname", self.interface
                ]
            else:
                cmd = [
                    "nmcli", "dev", "wifi", "connect", ssid,
                    "ifname", self.interface
                ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                # Wait for connection
                time.sleep(3)
                
                # Verify connection
                verify_cmd = ["nmcli", "-t", "-f", "GENERAL.STATE", "dev", "show", self.interface]
                verify_result = subprocess.run(verify_cmd, capture_output=True, text=True)
                
                if "100 (connected)" in verify_result.stdout:
                    return {"success": True, "message": "Connected successfully"}
                else:
                    return {"success": False, "message": "Connection failed"}
            else:
                return {"success": False, "message": result.stderr or "Connection failed"}
                
        except Exception as e:
            return {"success": False, "message": str(e)}
    
    def connect_to_saved(self, ssid):
        """Connect to a saved network"""
        try:
            cmd = ["nmcli", "connection", "up", ssid]
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                time.sleep(3)
                return {"success": True, "message": "Connected to saved network"}
            else:
                return {"success": False, "message": "Failed to connect to saved network"}
                
        except Exception as e:
            return {"success": False, "message": str(e)}
    
    def disable_hotspot(self):
        """Disable the hotspot"""
        try:
            subprocess.run(["nmcli", "connection", "down", "UAV-JETSON-CONFIG"],
                         capture_output=True)
            return True
        except:
            return False