"""Read-only clock acceptance and local Windows -> ROS calibration.

This module never changes a system clock, service or kernel rate. The existing
clock-recovery procedure remains a separate operational task.
"""
from __future__ import annotations
import ctypes
import json
import math
import platform
import select
import subprocess
import time


class Timeval(ctypes.Structure):
    _fields_=[('sec',ctypes.c_long),('usec',ctypes.c_long)]


class Timex(ctypes.Structure):
    _fields_=[('modes',ctypes.c_uint),('offset',ctypes.c_long),('freq',ctypes.c_long),
        ('maxerror',ctypes.c_long),('esterror',ctypes.c_long),('status',ctypes.c_int),
        ('constant',ctypes.c_long),('precision',ctypes.c_long),('tolerance',ctypes.c_long),
        ('time',Timeval),('tick',ctypes.c_long),('ppsfreq',ctypes.c_long),('jitter',ctypes.c_long),
        ('shift',ctypes.c_int),('stabil',ctypes.c_long),('jitcnt',ctypes.c_long),
        ('calcnt',ctypes.c_long),('errcnt',ctypes.c_long),('stbcnt',ctypes.c_long),
        ('tai',ctypes.c_int),('padding',ctypes.c_int*11)]


def kernel_rate():
    if platform.system()!='Linux' or platform.machine()!='x86_64':
        raise RuntimeError('this trial clock guard is for the audited x86_64 Linux host')
    value=Timex()  # modes=0: query only.
    lib=ctypes.CDLL(None,use_errno=True)
    if lib.adjtimex(ctypes.byref(value))<0:
        raise OSError(ctypes.get_errno(),'cannot query kernel clock rate')
    return dict(tick_us=value.tick,frequency_ppm=value.freq/65536)


MAXIMUM_RATE_PPM = 250.0  # Local maxdrift 100 + maxslewrate 100, plus rounding margin.


def accepted_rate(rate):
    # Linux expresses correction in BOTH the coarse tick and scaled frequency.
    # A healthy chronyd is allowed a small correction; old tick=10460 is 4.6%.
    total=(rate['tick_us']-10000)*100.+rate['frequency_ppm']
    if not math.isfinite(total) or abs(total)>MAXIMUM_RATE_PPM:
        raise RuntimeError('clock environment changed; stop the trial: '+json.dumps(rate))
    return dict(rate,total_rate_ppm=total)


def require_nominal_rate():
    rate=kernel_rate()
    return accepted_rate(rate)


class ClockGuard:
    def __init__(self):
        self.offset=None
        self.raw_anchor=None
        self.maximum_observed_rate_ppm=0.

    def check(self):
        require_nominal_rate()
        offset=time.time()-time.monotonic()
        if self.offset is not None and abs(offset-self.offset)>.05:
            raise RuntimeError('ROS wall/monotonic clock step during trial')
        self.offset=offset
        # CLOCK_MONOTONIC also includes NTP correction. RAW is required to
        # detect rate error independently of adjtimex's reported parameters.
        before=time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        mono=time.monotonic()
        after=time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        if after-before>.0002:return  # Scheduling makes this sample ambiguous.
        sample=((before+after)/2,mono)
        if self.raw_anchor is not None:
            raw_elapsed=sample[0]-self.raw_anchor[0]
            if raw_elapsed>=2.:
                rate=((sample[1]-self.raw_anchor[1])/raw_elapsed-1)*1e6
                self.maximum_observed_rate_ppm=max(self.maximum_observed_rate_ppm,abs(rate))
                if abs(rate)>500.:
                    raise RuntimeError('monotonic/raw clock rate exceeds 500ppm: '+str(rate))
                self.raw_anchor=sample
        else:self.raw_anchor=sample


def calibration_from_samples(samples,maximum_relative_rate_ppm=0.):
    if len(samples)<8:
        raise ValueError('at least eight paired clock samples required')
    if not math.isfinite(maximum_relative_rate_ppm) or maximum_relative_rate_ppm<0:
        raise ValueError('invalid clock rate bound')
    reference=samples[-1]['monotonic_s'] if maximum_relative_rate_ppm else None
    def allowance(row):
        return abs(reference-row['monotonic_s'])*maximum_relative_rate_ppm*1e-6 if reference is not None else 0.
    # Project intervals to one reference instant. The permitted, measured small
    # chrony rate correction makes a constant offset across the batch inexact.
    lo=max(row['lower_s']-allowance(row) for row in samples)
    hi=min(row['upper_s']+allowance(row) for row in samples)
    if not math.isfinite(lo+hi) or lo>hi:
        raise ValueError('clock bounds do not intersect; a constant clock offset is unproven')
    return dict(source_to_ros_offset_s=(lo+hi)/2,
        source_clock_uncertainty_s=(hi-lo)/2+.001,
        interval_s=[lo,hi],samples=samples,reference_monotonic_s=reference,
        maximum_relative_rate_ppm=maximum_relative_rate_ppm)


def _calibrate_local_windows_once():
    require_nominal_rate()
    # Precise FILETIME avoids DateTime.UtcNow's coarse tick quantization.
    script='''Add-Type -TypeDefinition 'using System.Runtime.InteropServices; public static class AsmcPreciseClock { [DllImport("kernel32.dll")] public static extern void GetSystemTimePreciseAsFileTime(out long value); }'; while ($null -ne [Console]::ReadLine()) { [long]$t=0; [AsmcPreciseClock]::GetSystemTimePreciseAsFileTime([ref]$t); [Console]::WriteLine($t) }'''
    process=subprocess.Popen(['powershell.exe','-NoProfile','-NonInteractive','-Command',script],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
    samples=[];discarded=0;restarts=0
    try:
        for i in range(80):
            left_begin=time.monotonic_ns();before=time.time_ns();left_end=time.monotonic_ns()
            process.stdin.write('ping\n');process.stdin.flush()
            if not select.select([process.stdout],[],[],10 if i==0 else 2)[0]:
                raise RuntimeError('Windows precise clock readback timed out')
            value=process.stdout.readline().strip()
            right_begin=time.monotonic_ns();after=time.time_ns();right_end=time.monotonic_ns()
            if not value.isdigit():raise RuntimeError('invalid Windows clock readback')
            # A descheduled local read is not evidence of a clock step. Both
            # endpoint reads must first be bounded by the monotonic clock.
            if left_end-left_begin>200000 or right_end-right_begin>200000:
                discarded+=1;continue
            elapsed=(right_begin+right_end-left_begin-left_end)//2
            if abs((after-before)-elapsed)>1000000:
                # Never intersect samples from different clock epochs. Obtain
                # a new complete stable set, or fail after a bounded retry.
                samples.clear();restarts+=1
                if restarts>3:raise RuntimeError('wall clock repeatedly stepped during calibration')
                continue
            windows_ns=(int(value)-116444736000000000)*100
            if i>=2:
                samples.append(dict(lower_s=(before-windows_ns)*1e-9,
                    upper_s=(after-windows_ns)*1e-9,rtt_s=elapsed*1e-9,
                    monotonic_s=(right_begin+right_end)*.5e-9))
            if len(samples)>=24:break
            time.sleep(.01)
    finally:
        process.stdin.close()
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=3)
    try:
        result=calibration_from_samples(samples,MAXIMUM_RATE_PPM)
    except ValueError as error:
        error.calibration_samples=samples
        raise
    result.update(method='paired Windows precise UTC / ROS host realtime',
        calibrated_at_unix_s=time.time(),kernel_rate=require_nominal_rate(),
        discarded_ambiguous_reads=discarded,clock_epoch_restarts=restarts)
    return result


def calibrate_local_windows():
    # A small Windows/ROS phase correction can occur DURING the paired batch.
    # Never widen its intervals or mix epochs to obtain an intersection. Throw
    # away that entire batch and obtain a new complete one, at most three times.
    # The operational profile's separately measured uncertainty still applies.
    rejected=[]
    for attempt in range(3):
        try:
            result=_calibrate_local_windows_once()
        except ValueError as error:
            if 'clock bounds do not intersect' not in str(error) or attempt==2:
                raise
            rejected.append(dict(error=str(error),samples=getattr(error,'calibration_samples',[])))
        else:
            result['rejected_calibration_epochs']=rejected
            return result
