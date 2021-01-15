.PHONY: help serve build install deploy

help:
	@cat README.md

serve:
	@bundle exec jekyll serve --watch

build:
	@bundle exec jekyll build

install:
	@bundle install

deploy: build
	@cp -R _site/. ../website-2020
