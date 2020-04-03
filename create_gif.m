function [] = create_gif(filename, gif_frame, h, TimeDelay)

    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    % Write to the GIF File
      if gif_frame == 1
       imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
      else
        imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime', TimeDelay);
      end
end
