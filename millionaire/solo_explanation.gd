extends Control

signal tutorial_finished

@onready var character = $Explain2/MeTalk
@onready var label = $Explain2/Label
@onready var voice_player = $Explain2/Speaking

var lines = [
	"Hello Player! Welcome to the Solo Challenge.",
	"To play, simply show your hand to the camera.",
	"Show 1 to 4 fingers to select your answer.",
	"Hold your hand steady for 3 seconds to lock it in!",
	"Are you ready to become a Millionaire?"
]

var current_line = 0
var is_animating = false

func _ready():
	label.text = ""
	if character:
		character.stop()

func start_explanation():
	current_line = 0
	show_next_line()

func show_next_line():
	# If we are already typing or out of lines, stop here
	if is_animating or current_line >= lines.size():
		if current_line >= lines.size():
			finish_tutorial_completely()
		return
		
	animate_text_now(lines[current_line])
	current_line += 1

func animate_text_now(full_text: String):
	is_animating = true
	label.text = "" 
	
	if character:
		character.play("default") # Mouth starts moving instantly
	
	# THE LOOP: Process each letter
	for i in range(full_text.length()):
		
		# 1. TRIGGER SOUND FIRST
		# We stop then play to force the 'blip' to restart for every letter
		if voice_player:
			voice_player.stop() 
			voice_player.play()
		
		# 2. SHOW THE LETTER IMMEDIATELY
		label.text += full_text[i]
		
		# 3. THE WAIT (Controls the speed of both sound and text)
		await get_tree().create_timer(0.06).timeout
	
	# --- AFTER THE LOOP FINISHES ---
	if character:
		character.stop()
		character.frame = 0 
	
	# Wait for reading time
	await get_tree().create_timer(2.0).timeout
	
	is_animating = false # Reset flag so the next line can start
	show_next_line()

func finish_tutorial_completely():
	self.hide()
	# Reference the GAME node and show it
	var game_ui = get_node("/root/Intro/SoloExplanation/GAME") 
	if game_ui:
		game_ui.show()
		# If your SingleGame script is on this node, tell it to start
		if game_ui.has_method("load_question"):
			game_ui.load_question(0)
	
	tutorial_finished.emit() 
	
