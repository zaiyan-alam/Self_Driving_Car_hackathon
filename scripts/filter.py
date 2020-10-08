def get_midpoint_peak(Ibw, row):
    
    #converting image to numpy array and taking a row L
    Ibw_array = np.array(Ibw)
    L = Ibw_array[row]
    L.shape
    
   
    #smoothening both arrays to filter the noise in the image we use a 3rd order Butterworth filter
    # Wn = 0.02, the cut-off frequency, acceptable values are from 0 to 1
    b, a = butter(3, 0.02)
    L_filt = filtfilt(b, a, L)

    all_peaks = find_peaks(L_filt, height = 0.5)   
    
    return all_peaks
