We used the same C++ main template and Makefile.
We did not change testmode# scripts.
The code can be compiled the same way as it was provided. 
Run make, chmod +x testmode#, and run testmode#

Similar to how the testbench is generated, 
we write a template called convtemplate.txt.
The code uses sed to replace tags in the text file.

In part 3, there are three templates:
convtemplate.txt - same as in previous parts
nettemplate.txt - top level module that instantiates 3 layers
commonModTemplate.txt - common modules such as memory that do not have tags/parameters

Thanks!

Jason and Evan