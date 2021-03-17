function mat2cpp(filename)

filetext = fileread(filename);

% exclude file extension
for i=length(filename):-1:1
    if (filename(i) == '.'), break; end
end
filename = filename(1:i-1);

% process text
filetext = strrep(filetext,'(1)','(0)');
filetext = strrep(filetext,'(2)','(1)');
filetext = strrep(filetext,'(3)','(2)');
filetext = strrep(filetext,'(4)','(3)');
filetext = strrep(filetext,'(5)','(4)');
filetext = strrep(filetext,'(6)','(5)');
filetext = strrep(filetext,'(7)','(6)');

filetext = strrep(filetext,'(1,','(0,');
filetext = strrep(filetext,'(2,','(1,');
filetext = strrep(filetext,'(3,','(2,');
filetext = strrep(filetext,'(4,','(3,');
filetext = strrep(filetext,'(5,','(4,');
filetext = strrep(filetext,'(6,','(5,');
filetext = strrep(filetext,'(7,','(6,');

filetext = strrep(filetext,',1)',',0)');
filetext = strrep(filetext,',2)',',1)');
filetext = strrep(filetext,',3)',',2)');
filetext = strrep(filetext,',4)',',3)');
filetext = strrep(filetext,',5)',',4)');
filetext = strrep(filetext,',6)',',5)');
filetext = strrep(filetext,',7)',',6)');

i = regexp(filetext,'\^');
while (~isempty(i))
   i = i(1);
   ss = filetext(i-4:i-1);
   filetext = [filetext(1:i-1) '*' ss filetext(i+2:end)];
   i = regexp(filetext,'\^');
end

% write to cpp file
cppfile = fopen([filename '.cpp'], 'w');
fprintf(cppfile, '%s', filetext);
fclose(cppfile);

end


