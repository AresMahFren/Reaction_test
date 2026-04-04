extends Control

signal tutorial_finished
signal explanation_finished


@onready var character = $Explain2/MeTalk
@onready var label = $Explain2/Label
@onready var voice_player = $Explain2/Speaking
@onready var skip_button = $Explain2/SkipButton
@onready var skip_timer = $Explain2/SkipTimer

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
	# Use find_parent to get the "intro" node, then look for "GAME"
	var game_node = get_tree().current_scene.find_child("GAME", true, false)
	if game_node:
		self.explanation_finished.connect(game_node._on_explanation_finished)
	else:
		print("Error: Could not find GAME node!")
	# Your other connections
	skip_timer.timeout.connect(_on_skip_timer_timeout)
	skip_button.pressed.connect(_on_skip_button_pressed)

func _on_skip_timer_timeout():
	# Show the button after 5 seconds
	skip_button.show()

func _on_skip_button_pressed():
	# 1. Stop audio/animation
	if voice_player: voice_player.stop()
	if character: character.stop()
	
	# 2. Since GAME is a child, we find it like this:
	var game_node = $GAME 
	
	if game_node:
		game_node.show() # This turns the visibility ON
		
		# Also find and hide the Choose menu (which IS a neighbor/sibling)
		var choose_node = get_node_or_null("../Choose")
		if choose_node:
			choose_node.hide()
			
		# 3. Trigger the start of the questions
		explanation_finished.emit()

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
	# 1. Hide ONLY the character and dialogue (the sibling/layer)
	# This keeps 'SoloExplanation' active so 'GAME' can be seen!
	$Explain2.hide() 
	
	# 2. Now find and show the quiz
	var game_ui = get_node("GAME") 
	if game_ui:
		game_ui.show()
		game_ui._display_question(0)
	else:
		print("Error: Could not find GAME node")
	
	tutorial_finished.emit()
	
