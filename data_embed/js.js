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
    var xhttpFramesList = new XMLHttpRequest();
    xhttpFramesList.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        const response = JSON.parse(this.responseText);
        let tbody = document.getElementById('receivedFrames');
        tbody.innerHTML = '';
        for (const frameInfo of response['received']) {
            let tr = document.createElement('tr');
            let td_p = document.createElement('td');
            td_p.innerHTML = frameInfo['packet'];
            tr.appendChild(td_p);
            let td_r = document.createElement('td');
            td_r.innerHTML = frameInfo['rssi'];
            tr.appendChild(td_r);
            let td_s = document.createElement('td');
            td_s.innerHTML = frameInfo['snr'];
            tr.appendChild(td_s);
            tbody.appendChild(tr);
        }
      }
    };
    xhttpFramesList.open("GET", "/received_list", true);
    xhttpFramesList.send();
};

function onFileChange(obj){
  var fileName = obj.value.split('\\');
  document.getElementById('file-input').innerHTML = fileName[fileName.length-1];
};

function updateFileUpload(event) {
  event.preventDefault();
  const file_input = document.getElementById("file-input");
  const file_progress = document.getElementById("file-progress");
  const data = new FormData(event.target);
  const xhr = new XMLHttpRequest();
  file_progress.classList.add("show");
  file_progress.value = 0;

  xhr.upload.onload = () => {
      window.location.reload();
  };

  // listen for `upload.error` event
  xhr.upload.onerror = () => {
      alert("Error!");
  }

  // listen for `upload.abort` event
  xhr.upload.onabort = () => {
      console.error('Upload cancelled.');
  }

  xhr.upload.onprogress = (event) => {
      // event.loaded returns how many bytes are downloaded
      // event.total returns the total number of bytes
      // event.total is only available if server sends `Content-Length` header
      console.log(`Uploaded ${event.loaded} of ${event.total} bytes`);
      let progress = 100 * (event.loaded / event.total);
      if (progress == 100){
          file_input.innerHTML = "Upgrade in progress. Please wait until page reloads!";
          file_progress.removeAttribute('value');
      } else {
        file_input.innerHTML = "Uploaded: " + Math.round(progress) +"%";
        file_progress.value = progress;
      }
  }

  xhr.open('POST', '/update');

  xhr.send(data);
}