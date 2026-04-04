extends Control

var server 
var listen_port := 4242

# --- DATA FROM PYTHON ---
var py_game_state = ""
var py_fingers = 0
var py_level = 0
var py_money = ""
var money_ladder = ["$100", "$1,000", "$10,000", "$100,000", "$1,000,000"]

# --- NEW AUDIO & TIMER VARIABLES ---
@onready var ingame_audio = $InGame
@onready var correct_audio = $Correct
@onready var gameover_audio = $GameOver
@onready var millionaire_audio = $Millionaire

# --- TIMER & STATE LOGIC ---
var lock_in_timer = 0.0
var required_time = 2.0 
var last_selected = -1
var is_showing_result = false
var is_game_over = false
var fist_timer = 0.0
var thumb_timer = 0.0
var time_left = 20.0
var timer_active = false

var fist_required_time = 3.0   
var thumb_required_time = 5.0  

# --- QUESTIONS ---
var quiz_data = [
	{"q": "What is 2 + 2?", "c": ["a.) 1", "b.) 2", "c.) 3", "d.) 4"], "a": 3},
	{"q": "Capital of France?", "c": ["a.) Berlin", "b.) Madrid", "c.) Paris", "d.) Rome"], "a": 2},
	{"q": "5 * 3?", "c": ["a.) 15", "b.) 10", "c.) 20", "d.) 25"], "a": 0},
	{"q": "Color of sky?", "c": ["a.) Blue", "b.) Green", "c.) Red", "d.) Yellow"], "a": 0},
	{"q": "Python is a?", "c": ["a.) Snake", "b.) Language", "c.) Car", "d.) Food"], "a": 1}
]

# --- UI REFERENCES ---
@onready var price_label = $Value/value
@onready var q_label = $Question/Questions
@onready var boxes = [$A, $B, $C, $D]
@onready var labels = [$A/a, $B/b, $C/c, $D/d]

func _ready():
	server = PacketPeerUDP.new() 
	server.bind(listen_port)
	
	# Stop all audio initially to prevent early playing
	ingame_audio.stop()
	correct_audio.stop()
	gameover_audio.stop()
	
	# Connect the signal for correct answers
	if not correct_audio.finished.is_connected(_on_correct_audio_finished):
		correct_audio.finished.connect(_on_correct_audio_finished)

func start_the_actual_game():
	# This is the most important line!
	self.show() 
	
	# Also ensure the specific UI elements are visible
	$Question.show()
	$Value.show()
	for b in boxes:
		b.show()
	
	# Your existing logic
	py_level = 0
	price_label.text = "$0"
	is_game_over = false
	ingame_audio.play()
	_display_question(0)

func _on_solo_explanation_explanation_finished():
	start_the_actual_game() # This will now work because it's in the same script!
	
func _on_explanation_finished():
	# This is where the magic happens!
	start_the_actual_game()

func _process(delta):
	if server.get_available_packet_count() > 0:
		var packet = server.get_packet().get_string_from_utf8()
		_handle_python_data(packet)

	if is_game_over:
		_handle_game_over_input(delta)
		return 

	# Countdown Logic
	if timer_active and not is_showing_result:
		time_left -= delta
		if time_left <= 0:
			_trigger_game_over("TIME'S UP!")

	# Normal Quiz Selection Logic
	if not is_showing_result:
		_handle_selection_logic(delta)

func _handle_python_data(data: String):
	var parts = data.split(",")
	if parts.size() >= 4:
		py_game_state = parts[0].strip_edges()
		py_fingers = int(parts[1])
		py_money = parts[3]
		if not is_game_over and not is_showing_result:
			_sync_ui_with_python()

func _display_question(index):
	if index < quiz_data.size():
		var data = quiz_data[index]
		q_label.text = data["q"]
		for i in range(4):
			labels[i].text = data["c"][i]
		
		# Reset and Start Timer
		time_left = 20.0
		timer_active = true
		
		# Play background music if not already playing
		if not ingame_audio.playing:
			ingame_audio.play()
	else:
		_victory()

func _validate_selection(index):
	if py_level >= quiz_data.size(): return
	
	is_showing_result = true
	timer_active = false 
	var correct_answer = quiz_data[py_level]["a"]
	
	if index == correct_answer:
		ingame_audio.stop()
		correct_audio.play() # Signal _on_correct_audio_finished will trigger next Q
		boxes[index].modulate = Color(0, 10.0, 0)
	else:
		_trigger_game_over("WRONG!")
		boxes[index].modulate = Color(10.0, 0, 0)
		boxes[correct_answer].modulate = Color(0, 10.0, 0)

func _on_correct_audio_finished():
	# Update the money label based on the level just completed
	if py_level < money_ladder.size():
		price_label.text = money_ladder[py_level]
	
	py_level += 1
	last_selected = -1
	is_showing_result = false
	
	# Check if we have more questions or if they just won the million
	if py_level < quiz_data.size():
		_display_question(py_level)
	else:
		_victory()

func _trigger_game_over(reason: String):
	if is_game_over: return
	is_game_over = true
	timer_active = false
	ingame_audio.stop()
	gameover_audio.play()
	q_label.text = reason + "\nFist (3s): Restart | Thumb (5s): Menu"

func _restart_game():
	is_game_over = false
	is_showing_result = false
	gameover_audio.stop()
	millionaire_audio.stop()
	py_level = 0
	fist_timer = 0.0
	thumb_timer = 0.0
	lock_in_timer = 0.0
	last_selected = -1
	for b in boxes: b.modulate = Color(1, 1, 1)
	price_label.text = "$0"
	_display_question(0)

func _victory():
	is_game_over = true
	timer_active = false
	ingame_audio.stop()
	
	# Play the big win sound!
	$Millionaire.play()
	
	# Update the UI
	price_label.text = "$1,000,000"
	q_label.text = "CONGRATULATIONS!\nYOU ARE A MILLIONAIRE!\n\nFist (3s): Restart | Thumb (5s): Menu"

# --- HELPER LOGIC FUNCTIONS ---

func _handle_game_over_input(delta):
	if py_game_state == "FIST":
		fist_timer += delta
		thumb_timer = 0.0
		q_label.text = "Restarting in: " + str(int(fist_required_time - fist_timer) + 1)
		if fist_timer >= fist_required_time: _restart_game()
	elif py_game_state == "THUMBS_UP":
		thumb_timer += delta
		fist_timer = 0.0
		q_label.text = "Menu in: " + str(int(thumb_required_time - thumb_timer) + 1)
		if thumb_timer >= thumb_required_time: get_tree().change_scene_to_file("res://intro.tscn")
	else:
		fist_timer = 0.0
		thumb_timer = 0.0

func _handle_selection_logic(delta):
	if py_fingers >= 1 and py_fingers <= 4:
		var current_selection = py_fingers - 1
		if current_selection == last_selected:
			lock_in_timer += delta
			boxes[current_selection].modulate = Color(lock_in_timer, lock_in_timer, 10.0)
			if lock_in_timer >= required_time:
				_validate_selection(current_selection)
				lock_in_timer = 0.0
		else:
			lock_in_timer = 0.0
			last_selected = current_selection
			_sync_ui_with_python()
	else:
		lock_in_timer = 0.0
		last_selected = -1
		_sync_ui_with_python()

func _sync_ui_with_python():
	if is_showing_result: return
	for b in boxes: b.modulate = Color(1, 1, 1)
	if py_fingers >= 1 and py_fingers <= 4:
		boxes[py_fingers - 1].modulate = Color(0, 0.5, 10.0)
