function! ConvertLogsToRead()
    silent! set ft=json
    silent! set nowrap
    silent! %s/\(^.*;\)/\1\r/
    silent! %s/[{,]/&\r/g
    silent! %s/[}]/\r&/g
    silent! normal gg=G
    silent! normal gg
endfunction


command! FormatLogs call ConvertLogsToRead()

