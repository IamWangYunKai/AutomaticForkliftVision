function condbuf(max_cnt, sleep_time)
  local cur_cnt = 0

  return function (cond)
    if cond then
      cur_cnt = cur_cnt + 1
      if cur_cnt >= max_cnt then
        cur_cnt = 0
        return true
      else
        SleepL(sleep_time)
      end
    else
      cur_cnt = cur_cnt - 1
      if cur_cnt < 0 then
        cur_cnt = 0
      end
    end
    return false
  end
end