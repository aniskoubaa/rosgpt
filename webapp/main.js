<!DOCTYPE html>
<html>

<head>
  <title>RobotGPT</title>
  <!-- Add Material-UI CSS -->
  <link rel="stylesheet" href="https://fonts.googleapis.com/css?family=Roboto:300,400,500,700&display=swap">
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/materialize/1.0.0/css/materialize.min.css">
  <link rel="stylesheet" type="text/css" href="style.css">
</head>

<body>
  <div class="container">
    <div class="row">
      <div class="col s12">
        <h1>RobotGPT</h1>
        <p>Click the "Start Recording" button and speak your command. Then click the "Stop Recording" button to convert it
          to speech. Finally, click the "Send to Robot" button to send the voice command to the robot.</p>
      </div>
    </div>
    <div class="row">
      <form class="col s12">
        <div class="row">
          <div class="input-field col s6">
            <input placeholder="IP Address" id="ipAddress" type="text" class="validate" value="192.168.191.147">
            <label for="ipAddress">IP Address</label>
          </div>
          <div class="input-field col s6">
            <input placeholder="Port Number" id="portNumber" type="text" class="validate" value="5000">
            <label for="portNumber">Port Number</label>
          </div>
        </div>
      </form>
    </div>
    <div class="row">
      <div class="col s12 m4">
        <button id="startBtn" class="btn-large waves-effect waves-light">
          <i class="material-icons left">mic</i>
        </button>
      </div>
      <div class="col s12 m4">
        <button id="stopBtn" class="btn-large waves-effect waves-light">
          <i class="material-icons left">stop</i>
        </button>
      </div>
      <div class="col s12 m4">
        <button id="sendBtn" class="btn-large waves-effect waves-light">
          <i class="material-icons left">send</i>
        </button>
      </div>
    </div>
    <div class="row">
      <div id="output" class="col s12"></div>
    </div>
  </div>

  <footer>
    <div class="container">
      <p>&copy; Anis Koubaa - April 2023</p>
    </div>
  </footer>

  <!-- Add Material-UI JS -->
  <script src="https://code.jquery.com/jquery-3.6.0.min.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/materialize/1.0.0/js/materialize.min.js"></script>
  <script src="script.js"></script>
</body>

</html>
