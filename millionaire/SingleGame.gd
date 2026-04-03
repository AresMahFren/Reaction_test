extends Control

var server 
var listen_port := 4242

# --- DATA FROM PYTHON ---
var py_game_state = ""
var py_fingers = 0
var py_level = 0
var py_money = ""

# --- QUESTIONS ---
var quiz_data = [
	{"q": "What is 2 + 2?", "c": ["1", "2", "3", "4"]},
	{"q": "Capital of France?", "c": ["Berlin", "Madrid", "Paris", "Rome"]},
	{"q": "5 * 3?", "c": ["15", "10", "20", "25"]},
	{"q": "Color of sky?", "c": ["Blue", "Green", "Red", "Yellow"]},
	{"q": "Python is a?", "c": ["Snake", "Language", "Car", "Food"]}
]

# --- UI REFERENCES ---
@onready var price_label = $Value/value
@onready var q_label = $Question/Questions
@onready var boxes = [$A, $B, $C, $D]
@onready var labels = [$A/a, $B/b, $C/c, $D/d]

func _ready():
	# We force the creation of the object here
	server = PacketPeerUDP.new() 
	
	# Now 'server' is a real object and 'listen' will exist
	var error = server.bind(listen_port)
	if error != OK:
		print("UDP Error: ", error)
	else:
		print("UDP: Connected to Python on port ", listen_port)
	
	_display_question(0)

func _process(_delta):
	if server.get_available_packet_count() > 0:
		var packet = server.get_packet().get_string_from_utf8()
		_handle_python_data(packet)

func _handle_python_data(data: String):
	var parts = data.split(",")
	if parts.size() >= 4:
		var old_level = py_level
		py_game_state = parts[0]
		py_fingers = int(parts[1])
		py_level = int(parts[2])
		py_money = parts[3]
		
		if py_level != old_level:
			_display_question(py_level)
		_sync_ui_with_python()

func _display_question(index):
	if index < quiz_data.size():
		var data = quiz_data[index]
		q_label.text = data["q"]
		for i in range(4):
			labels[i].text = data["c"][i]

func _sync_ui_with_python():
	price_label.text = py_money
	for b in boxes:
		b.self_modulate = Color(1, 1, 1) # Reset to white
	
	# Blue selection glow
	if py_fingers >= 1 and py_fingers <= 4:
		boxes[py_fingers - 1].self_modulate = Color(0, 2, 5)
