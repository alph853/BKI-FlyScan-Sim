let selectedNetwork = null;
let savedNetworks = [];

async function loadNetworks() {
    try {
        // Get saved networks first
        const savedResponse = await fetch('/api/saved-networks');
        const savedData = await savedResponse.json();
        savedNetworks = savedData.saved_networks || [];
        
        // Get available networks
        const response = await fetch('/api/networks');
        const data = await response.json();
        
        displayNetworks(data.networks);
    } catch (error) {
        showStatus('Failed to load networks', 'error');
    }
}

function displayNetworks(networks) {
    const container = document.getElementById('networks-list');
    container.innerHTML = '';
    
    networks.forEach(network => {
        const div = document.createElement('div');
        div.className = 'network-item';
        div.onclick = () => selectNetwork(network);
        
        const isSaved = savedNetworks.includes(network.ssid);
        
        div.innerHTML = `
            <div class="network-info">
                <div class="network-ssid">
                    ${network.ssid}
                    ${isSaved ? '<span class="saved-badge">SAVED</span>' : ''}
                </div>
                <div class="network-details">
                    ${network.band} • ${network.secured ? '🔒 Secured' : '🔓 Open'}
                </div>
            </div>
            <div class="signal-indicator">
                ${getSignalBars(network.signal)}
            </div>
        `;
        
        container.appendChild(div);
    });
    
    document.getElementById('loading').style.display = 'none';
    document.getElementById('networks-container').style.display = 'block';
}

function getSignalBars(signal) {
    if (signal >= 80) return '▂▄▆█';
    if (signal >= 60) return '▂▄▆_';
    if (signal >= 40) return '▂▄__';
    return '▂___';
}

function selectNetwork(network) {
    selectedNetwork = network;
    
    const isSaved = savedNetworks.includes(network.ssid);
    
    if (network.secured && !isSaved) {
        // Show password modal
        document.getElementById('selected-ssid').textContent = network.ssid;
        document.getElementById('password-modal').style.display = 'flex';
        document.getElementById('password-input').focus();
    } else {
        // Connect directly (open network or saved network)
        connect();
    }
}

async function connect() {
    const password = document.getElementById('password-input').value;
    
    closeModal();
    showStatus('Connecting...', 'info');
    
    try {
        const response = await fetch('/api/connect', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                ssid: selectedNetwork.ssid,
                password: password
            })
        });
        
        const result = await response.json();
        
        if (result.success) {
            showStatus('Connected successfully! The hotspot will now be disabled.', 'success');
            setTimeout(() => {
                showStatus('You can now connect to the same network to access the Jetson.', 'success');
            }, 3000);
        } else {
            showStatus(result.message || 'Connection failed', 'error');
        }
    } catch (error) {
        showStatus('Connection error', 'error');
    }
}

function closeModal() {
    document.getElementById('password-modal').style.display = 'none';
    document.getElementById('password-input').value = '';
}

function showStatus(message, type) {
    const statusEl = document.getElementById('status-message');
    statusEl.textContent = message;
    statusEl.className = `status-message ${type}`;
    statusEl.style.display = 'block';
}

// Handle Enter key in password input
document.getElementById('password-input').addEventListener('keypress', (e) => {
    if (e.key === 'Enter') {
        connect();
    }
});

// Load networks when page loads
document.addEventListener('DOMContentLoaded', () => {
    loadNetworks();
    // Refresh every 10 seconds
    setInterval(loadNetworks, 10000);
});