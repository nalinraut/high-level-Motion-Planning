A fork of

#IML Internal Repository
Kris Hauser  

by CIBR Lab @ WPI


##[Check out the lab wiki](https://github.com/duke-iml/iml-internal/wiki)
It has in depth guides to common lab tools and interfaces. Especially good for anyone new.

##Guidelines
- The main directory should contain code / data that is of general use to
  other members of the lab, or is used in a paper. 
- Document your contributions if possible, or at least include a README
  file in your contribution that indicates the purpose of the files and
  their author(s).
- Documentation for the lab equipment is found in the
  "EquipmentManualsAndTraining" folder.
- Everyone's machine configuration will be somewhat different, so please try
  to avoid committing insignificant changes to configuration files (like
  Makefiles).  The .gitignore file is good for this.

##Help
- **Personal Computers:** The simplest setup is [Github for Windows](https://windows.github.com/) or [Github for Mac](https://mac.github.com/). It is still recomended to [learn some terminal commands](http://rogerdudler.github.io/git-guide/) though.
- **Shared/lab/other computer:** To commit under a different name than the computer is set up for, add the ```--author= Name <email>``` options to your commit. For example, to commit with a message (which you should) and under your account (which will be found via your email), commit by typing:  
```git commit -m "Your Message" --author="First Last <email@gmail.com>"```

##Issues/TODOs

- We are noıw keeping track of  what to do in the issues.
- This branch is currently focusing on updating the existing iml-code on the usage of messages to be abel to build iwth the up to date ros pkgs.
