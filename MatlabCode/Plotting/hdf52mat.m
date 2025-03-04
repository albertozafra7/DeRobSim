function []= hdf52mat(fullFileName)
    [times, Matlab, Unity, unity_keys] = customHDF5_read(fullFileName);
    
    shortFileName = split(fullFileName,'.');
    shortFileName = string(shortFileName(1));
    
    save(strcat(shortFileName,".mat"),"times","Matlab","Unity","unity_keys",'-mat');
end