.PHONY: init docs test

# installs pipenv and builds the project
init:
	sudo pip install pipenv --upgrade
	pipenv install --dev

# builds the documentation
docs:
	pipenv run python setup.py docs

# runs the unit test suite
test:
	pipenv run pytest
