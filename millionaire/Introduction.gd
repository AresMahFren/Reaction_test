extends Control

@onready var video = $Start
@onready var profile_image = $Play
@onready var menu = $Choose
@onready var loading_screen = $Loading

# UPDATED PATHS: We tell Godot to look inside the "Play" node
@onready var intro_music = $Play/intro  
@onready var select_sfx = $Play/enter
@onready var solo_icon = $Choose/SoloButton
@onready var loading_timer = $Timer

var state = "VIDEO" 
var game_running = false
var is_game_active = false
var udp := PacketPeerUDP.new()
static var has_played_video_once = false

func _ready():
	# Connect signals (Keep your existing ones)
	video.finished.connect(_on_video_finished)
	loading_timer.timeout.connect(_on_loading_finished)

	if has_played_video_once:
		# --- SKIP TO MENU ---
		video.hide()
		video.stop()
		profile_image.hide()
		menu.show()
		state = "MENU"
		if intro_music.stream:
			intro_music.play()
	else:
		# --- FIRST TIME STARTUP ---
		profile_image.hide()
		menu.hide()
		loading_screen.hide()
		video.play()
		# Now set it to true so it skips next time
		has_played_video_once = true
	
		# Connect the hover signals
	$Choose/SoloButton.mouse_entered.connect(_on_solo_hover)
	$Choose/SoloButton.mouse_exited.connect(_on_hover_off)
	
	# Connect the click signals
	$Choose/SoloButton.pressed.connect(_on_solo_selected)
	

func _input(event):
	if event.is_action_pressed("ui_accept"):
		match state:
			"VIDEO":
				skip_video()
			"PROFILE":
				play_select_and_show_menu()
			"MENU":
				# Instead of going straight to loading, show the explanation
				start_solo_explanation()
				

func _on_solo_hover():
	if solo_icon:
		solo_icon.scale = Vector2(1.1, 1.1)
		solo_icon.modulate = Color(1.2, 1.2, 1.2) # Makes it glow



func _on_hover_off():
	if solo_icon:
		solo_icon.scale = Vector2(1, 1)
		solo_icon.modulate = Color(1, 1, 1)

func start_solo_explanation():
	state = "EXPLAINING"
	menu.hide()
	$SoloExplanation.show()
	# This triggers the 'Talking' script we wrote earlier
	$SoloExplanation.show_next_line()

func start_main_game():
	# Stop the intro from listening so SingleGame.gd can take over port 4242 [cite: 5, 8]
	game_running = false 
	if udp.is_bound():
		udp.close()
	print("Handing over control to SingleGame script.")

func skip_video():
	video.stop()
	_on_video_finished()

func _on_video_finished():
	if state == "VIDEO":
		state = "PROFILE"
		video.hide()
		profile_image.show()
		# Make sure the 'Stream' is not empty in the inspector!
		if intro_music.stream:
			intro_music.play()

func play_select_and_show_menu():
	state = "MENU"
	if select_sfx.stream:
		select_sfx.play()
	profile_image.hide()
	menu.show()

func _on_solo_selected():
	$Play/enter.play()
	$Choose.hide()
	
	# 1. Show Loading Screen
	$Loading.show()
	$Play/intro.stop()
	
	# 2. Start the timer - ONLY the signal (_on_loading_finished) should handle the rest
	$Timer.start(7.0) 
	# DELETE EVERYTHING AFTER THIS LINE IN THIS FUNCTION

func _process(_delta):
	if game_running:
		if udp.get_available_packet_count() > 0:
			var packet = udp.get_packet().get_string_from_utf8()
			var data = packet.split(",") 
			
			if data.size() >= 2:
				var python_state = data[0]
				var fingers = data[1].to_int()
				_update_game_ui(python_state, fingers)

func _update_game_ui(p_state, p_fingers):
	# Example: Make the Solo button glow when 1 finger is shown
	if p_state == "MENU":
		if p_fingers == 1:
			_on_solo_hover()
		elif p_fingers == 2:
			_on_hover_off()

func start_loading():
	state = "LOADING"
	if select_sfx.stream:
		select_sfx.play()
	
	menu.hide()
	loading_screen.show()
	intro_music.stop() # Stop music during loading
	
	loading_timer.start(7.0) # Starts the 7 second countdown

func _on_loading_finished():
	$Loading.hide()
	$SoloExplanation.show()
	# This starts the loop. The explanation script will handle the rest!
	$SoloExplanation.start_explanation() 
	start_main_game()
