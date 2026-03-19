extends Control

@onready var video = $Start
@onready var profile_image = $Play
@onready var menu = $Choose
@onready var loading_screen = $Loading

# UPDATED PATHS: We tell Godot to look inside the "Play" node
@onready var intro_music = $Play/intro  
@onready var select_sfx = $Play/enter
@onready var loading_timer = $Timer

var state = "VIDEO" 

func _ready():
	# Initial setup: hide everything except the video
	profile_image.hide()
	menu.hide()
	loading_screen.hide()
	
	video.play()
	
	# Connect signals
	video.finished.connect(_on_video_finished)
	loading_timer.timeout.connect(_on_loading_finished)

func _input(event):
	if event.is_action_pressed("ui_accept"):
		match state:
			"VIDEO":
				skip_video()
			"PROFILE":
				play_select_and_show_menu()
			"MENU":
				start_loading()

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

func start_loading():
	state = "LOADING"
	if select_sfx.stream:
		select_sfx.play()
	
	menu.hide()
	loading_screen.show()
	intro_music.stop() # Stop music during loading
	
	loading_timer.start(7.0) # Starts the 7 second countdown

func _on_loading_finished():
	print("Loading complete! Ready for gameplay.")
	# This is where you will eventually switch to your Solo or Duel scene
