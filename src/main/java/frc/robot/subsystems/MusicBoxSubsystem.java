package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MusicConstants;
import frc.robot.Constants.OuttakeConstants;

public class MusicBoxSubsystem extends SubsystemBase {
    private final TalonFX leadInstrument = new TalonFX(OuttakeConstants.motorID, "can");
    private final Orchestra orchestra = new Orchestra();

    private int selectedSongIndex = 0;
    private String loadedSong = "";
    private boolean playing = false;

    public MusicBoxSubsystem() {
        orchestra.addInstrument(leadInstrument);

        if (hasSongs()) {
            loadSelectedSong();
        }
    }

    public boolean canControlMusic() {
        return DriverStation.isDisabled();
    }

    public boolean hasSongs() {
        return MusicConstants.songFiles.length > 0;
    }

    public void loadSelectedSong() {
        String desiredSong = MusicConstants.songFiles[selectedSongIndex];
        if (desiredSong.equals(loadedSong)) {
            return;
        }

        StatusCode status = orchestra.loadMusic(desiredSong);
        if (status != StatusCode.OK) {
            return;
        }

        loadedSong = desiredSong;
    }

    public void playLoadedSong() {
        if (loadedSong.isEmpty()) {
            loadSelectedSong();
        }

        if (loadedSong.isEmpty()) {
            return;
        }

        StatusCode status = orchestra.play();
        if (status != StatusCode.OK) {
            return;
        }

        playing = true;
    }

    public void pauseSong() {
        StatusCode status = orchestra.pause();
        if (status != StatusCode.OK) {
            return;
        }

        playing = false;
    }

    public Command selectNextSong() {
        return this.runOnce(() -> {
            if (!canControlMusic() || !hasSongs()) {
                return;
            }

            selectedSongIndex = (selectedSongIndex + 1) % MusicConstants.songFiles.length;
            playing = false;
            loadSelectedSong();
        });
    }

    public Command selectPreviousSong() {
        return this.runOnce(() -> {
            if (!canControlMusic() || !hasSongs()) {
                return;
            }

            selectedSongIndex = (selectedSongIndex - 1 + MusicConstants.songFiles.length)
                    % MusicConstants.songFiles.length;
            playing = false;
            loadSelectedSong();
        });
    }

    public Command togglePlayPause() {
        return this.runOnce(() -> {
            if (!canControlMusic() || !hasSongs()) {
                return;
            }

            if (playing) {
                pauseSong();
            } else {
                loadSelectedSong();
                playLoadedSong();
            }
        });
    }
}
