clear all, clc
pixel = 400;
width = 9;
height = 7;

chessboard=zeros(pixel*height, pixel*width);
% test=ones(height,width);

for i=1:width*height
    r=ceil(i/width)
    c=mod(i,width)
    if(c==0)    c=9;
    end
    
    if mod(i,2)==1
        chessboard(pixel*(r-1)+1:pixel*r,1+pixel*(c-1):pixel*c)=zeros(pixel,pixel);
    else
        chessboard(pixel*(r-1)+1:pixel*r,1+pixel*(c-1):pixel*c)=ones(pixel,pixel);
    end
end

imshow(chessboard)
imwrite(chessboard,'chess.png')