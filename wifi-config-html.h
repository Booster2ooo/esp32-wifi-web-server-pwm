const char CONFIG_page[] PROGMEM = R"=====(
<!DOCTYPE HTML>
<html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>WiFi Configuration</title>
    <style>*{padding:0;margin:0;font-family:sans-serif}html{width:100%;height:100%}body{width:100%;height:100%;background-color:#1c1c1c;margin:auto;text-align:center;color:#8f8}h1,h2,h3,h4,h5,h6{margin-top:5rem}form{margin:5rem auto;width:300px}label{display:flex;justify-content:space-between;margin-bottom:.5rem}input{background-color:#181818;color:#cecece;padding:.25rem;border:0 none}button{background-color:#8f8;color:#181818;padding:.5rem;border:0 none;border-radius:.25rem}</style>
  </head>
  <body>
    <h1>WiFi configuration</h1>
    <form action="/save-credentials" method="POST">
      <label for="ssid">
        SSID
        <input type="text" name="ssid" id="ssid" placeholder="SSID" value="%SSID%" />
      </label>
      <label for="password">
        Password
        <input type="password" name="password" id="password" placeholder="password" value="%PASSWORD%" />
      </label>
      <button type="submit">Save</button>
    </form>
  </body>
</html>
)=====";