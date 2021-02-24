/**/
function scanWifi() {
  let scanBtn = document.getElementById('scan_wifi_btn');
  let wifiListContainer = document.getElementById("wifi_list");
  wifiListContainer.innerHTML = 'Scanning...';
  scanBtn.disabled = true;
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    scanBtn.disabled = false;
    if (this.readyState == 4 && this.status == 200) {
      wifiListContainer.innerHTML = this.responseText;
      const networks_found_list = document.querySelector('#networks_found_list');

      networks_found_list.addEventListener('change', event => {
        document.getElementById('wifi_ssid').value = networks_found_list.value;
      });
    }
  };
  xhttp.open("GET", "/scan_wifi", true);
  xhttp.send();
}

window.onload = function () {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        const response = JSON.parse(this.responseText);
        for (const [key, value] of Object.entries(response)) {
          let element = document.getElementById(key);
          if (element){
            if (element.type && element.type == "checkbox"){
              element.checked = value;
            } else {
              element.value = value;
            }
          }
        }
      }
    };
    xhttp.open("GET", "/cfg", true);
    xhttp.send();
};