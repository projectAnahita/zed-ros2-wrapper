# Source global definitions
if [ -f /etc/bashrc ]; then
    . /etc/bashrc
fi

# Source aliases
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# Set prompt
PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '