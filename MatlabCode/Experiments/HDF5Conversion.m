files = dir('*/*.h5');

for(i = 1:length(files))
    hdf52mat(fullfile(files(i).folder, files(i).name));
    disp("Converted " + files(i).name);
end