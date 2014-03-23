import sys
import os
import datetime
import argparse
import re


#######################################################
## Default values
## MY_NAME, COURSE, and INSTRUCTOR may be left as empty
##   strings.  MY_NAME may be a list of strings or a
##   single string.
#######################################################
MY_NAME = 'Huston Bokinsky'
COURSE = 'CSCE 574 - Robotics'
INSTRUCTOR = "Dr. O'Kane"

#########################################################
## Recognized file extensions may be added to as needed.
#########################################################
RECOGNIZED_EXTENSIONS = {
'cpp' : { 'top' : '/' + '*' * 50,
          'mid' : ' * ',
          'end' : ' ' + '*' * 50 + '/' },

'c'   : { 'top' : '/' + '*' * 50,
          'mid' : ' * ',
          'end' : ' ' + '*' * 50 + '/' },

'h'   : { 'top' : '/' + '*' * 50,
          'mid' : ' * ',
          'end' : ' ' + '*' * 50 + '/' },

'py'  : { 'top' : '#' * 50,
          'mid' : '## ',
          'end' : '#' * 50 },

'launch': { 'top' : '<!--\n    ' + '*' * 45,
            'mid' : '     ',
            'end' : '    ' + '*' * 45 + '\n  -->'},

'xml' : { 'top' : '<!--\n    ' + '*' * 45,
          'mid' : '     ',
          'end' : '    ' + '*' * 45 + '\n  -->'},
}

#######################################################
## Regular expressions for eliminating previous headers
#######################################################
undo_re = {
        'c' : re.compile("\s*/\*.+?\*/", re.DOTALL),
        'cpp' : re.compile("\s*/\*.+?\*/", re.DOTALL),
        'h' : re.compile("\s*/\*.+?\*/", re.DOTALL),
        'py' : re.compile("^(\s*#.*\r?\n)*"),
        'launch' : re.compile("^\s*<!--.*?-->", re.DOTALL),
        'xml' : re.compile("^\s*<!--.*?-->", re.DOTALL),
        }

#######################################################
## Get invocation arguments
#######################################################
parser = argparse.ArgumentParser()
parser.add_argument("filedir",
        type=str,
        help="target file or directory of files to which to prepend header")
parser.add_argument('-u', '--undo',
        action="store_true",
        help="remove existing header (if any) before prepending new one")
parser.add_argument('-t', '--test',
        type=int,
        choices=[1, 2],
        help="find files that would be modified without making changes. \
1 = just print filenames; 2 = print filenames and file head.")
parser.add_argument('-a', '--asst', 
        help="assignment name to include")
parser.add_argument('-d', '--description', 
        help="assignment description")
parser.add_argument('-n', '--name', 
        nargs='*', 
        type=str,
        help="name(s) of file author(s); defaults to '{}'".format(MY_NAME))
parser.add_argument('-x', '--exclude',
        nargs='*',
        type=str,
        help='files to exclude from modification.  Get these from \
output of --test')
parser.add_argument('-c', '--course',
        help="course name; defaults to '{}'".format(COURSE))
parser.add_argument('-i', '--instructor', 
        help="instructor name; defaults to '{}'".format(INSTRUCTOR))
args=parser.parse_args()


# get filename/dirname
filename = os.path.basename(args.filedir)
dirname  = (os.path.dirname(args.filedir) if
            os.path.dirname(args.filedir) else
            os.getcwd())   

# check for assignment
assignment = args.asst if args.asst else ''
if not assignment and not args.test:
    print "No assignment found.  Don't forget to add one to header."

# check for and build description string
dl = args.description.split() if args.description else ''
description = ''
pos = 0
for word in dl:
    if pos + len(word) >= 50:
        description += "\n{mid}" + ' ' * 14
        pos = 0
    description += word + ' '
    pos += len(word) + 1
if not description and not args.test:
    print "No description found.  Don't forget to add one to header."

# get date
today = datetime.date.today()
day = today.strftime("%d %B, %Y")

# walk through target files
for root, dirs, files in (os.walk(args.filedir) if not
                          filename else
                          [(dirname, None, [filename])]):

    for filename in files:
        # check if recognized filetype
        ext = filename.split('.')[-1]
        if ext not in RECOGNIZED_EXTENSIONS.keys():
            print ("Skipping %s -- unrecognized file extension." %
                    os.path.join(root, filename))
            continue

        fullpath = os.path.join(root, filename)

        # check against exclude list (if any)
        if args.exclude and fullpath in args.exclude:
            print "Skipping %s -- excluded at invocation." % fullpath
            continue

        # try to open file
        try:
            f = open(fullpath, 'r')
        except IOError:
            print "Skipping %s -- unable to read." % fullpath
            continue
 
        # if test, just print name [and head] and continue
        if args.test:
            if args.test >= 1:
                print "\nFound %s" % fullpath
            if args.test >= 2:
                for line in [line for line in f][:10]:
                    print line.rstrip()
            f.close()
            continue

        # get original file text, discarding old header if called for
        temptext = f.read()
        f.close()
        if args.undo:
            temptext = undo_re[ext].sub('', temptext, count=1)
            
        # gather values
        values = dict()
        values['name'] = args.name if args.name else MY_NAME
        values['filename'] = filename
        values['date'] = day
        values['course'] = args.course if args.course else COURSE 
        values['instructor'] = (args.instructor if
                args.instructor else INSTRUCTOR)
        values['assignment'] = assignment
        values['description'] = description.format(
                **RECOGNIZED_EXTENSIONS[ext])
        values['top'] = RECOGNIZED_EXTENSIONS[ext]['top']
        values['mid'] = RECOGNIZED_EXTENSIONS[ext]['mid']
        values['end'] = RECOGNIZED_EXTENSIONS[ext]['end']

        # build new header string
        #  Currently only handles possible list of author names.
        #  This functionality should be easy to extend to 
        #  course and instructor if necessary by modifying those
        #  sections similarly to the name section
        header  = "{top}\n"
        header += "{mid}Name:         {name%s}\n" % ('[0]' if
                isinstance(values['name'], list) else '')
        if isinstance(values['name'], list):
            for i in range(1, len(values['name'])):
                header += "{mid}              {name[%d]}\n" % i
        header += "{mid}File:         {filename}\n"
        header += "{mid}Assignment:   {assignment}\n"
        header += "{mid}Date:         {date}\n"
        header += "{mid}Course:       {course}\n"
        header += "{mid}Instructor:   {instructor}\n"
        header += "{mid}Description:  {description}\n"
        header += "{mid}\n"
        header += "{end}\n\n"

        # write
        with open(fullpath, 'w') as f:
            f.write(header.format(**values))
            f.write(temptext.strip())

# if only testing, print friendly reminder at end
if args.test:
    print "\nTesting only.  No files modified."

