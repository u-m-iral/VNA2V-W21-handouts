# Handouts Jekyll website

## Quickstart

1. Install [RVM](https://rvm.io/install) - Ruby Version Manager and a Ruby environment  
If you are impatient, try run the following script:  

``` bash
gpg --keyserver hkp://pool.sks-keyservers.net --recv-keys 409B6B1796C275462A1703113804BB82D39DC0E3 7D2BAF1CF37B13E2069D6956105BD0E739499BDB
\curl -sSL https://get.rvm.io | bash -s stable --auto-dotfiles
```

After running, try run `rvm system`. If you see a message similar to something below:  

``` bash
RVM is not a function, selecting rubies with 'rvm use ...' will not work.
```

then you need to set the terminal you are using to a login shell. See [this](https://rvm.io/integration/gnome-terminal) and [this](https://stackoverflow.com/questions/9336596/rvm-installation-not-working-rvm-is-not-a-function).
After you setting it to a login shell, reopen the terminal and run the following to set up a Ruby environment:  

``` bash
rvm install ruby-2.7.0
rvm --default use ruby-2.7.0
```

2. Install Jekyll and bundler
```bash
gem install jekyll bundler
```
3. Clone this repository
4. Install dependencies
```bash
make install
```

## Run local webserver

To run local webserver simply run

```bash
make serve
```

<!-- ## Deploy to public github
Clone the public website repo at `website-2020` on the same level as this repo. Then, after you `make build`, run `make deploy`. This will copy all files in `_site` to the public site repo. -->
