/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   get_next_line.c                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: cari <cari@student.42.fr>                  +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/12/01 02:41:11 by cari              #+#    #+#             */
/*   Updated: 2025/03/16 00:53:10 by cari             ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static char	*start_str_generator(char *start_str, char *tmp)
{
	char	*new_start_str;

	new_start_str = ft_strjoin(start_str, tmp);
	free(start_str);
	return (new_start_str);
}

static char	*trim_buffer(char	*buf)
{
	char	*new_buff;
	int		i;
	int		j;

	i = 0;
	while (buf[i] && buf[i] != '\n')
		i++;
	if (buf[i] == '\0')
	{
		free(buf);
		return (NULL);
	}
	i += (buf[i] == '\n');
	new_buff = (char *)malloc(1 + ft_strlen(buf) - i);
	if (!new_buff)
		return (NULL);
	j = 0;
	while (buf[i + j])
	{
		new_buff[j] = buf[i + j];
		j++;
	}
	new_buff[j] = '\0';
	free(buf);
	return (new_buff);
}

static char	*trim_line(char *line)
{
	int		i;
	char	*new_line;

	if (!line || !line[0])
		return (NULL);
	i = 0;
	while (line[i] && line[i] != '\n')
		i++;
	if (line[i] == '\n')
		i++;
	new_line = (char *)malloc(i + 1);
	if (!new_line)
		return (NULL);
	i = 0;
	while (line[i] && line[i] != '\n')
	{
		new_line[i] = line[i];
		i++;
	}
	if (line[i] == '\n')
		new_line[i++] = '\n';
	new_line[i] = '\0';
	return (new_line);
}

char	*get_next_line(int fd)
{
	char		*tmp;
	int			fd_ret;
	static char	*start_str;

	if (fd < 0 || BUFFER_SIZE <= 0)
		return (NULL);
	fd_ret = 1;
	tmp = (char *)malloc(BUFFER_SIZE + 1);
	if (!tmp)
		return (NULL);
	while (ft_strchr(start_str, '\n') == NULL && fd_ret != 0)
	{
		fd_ret = read(fd, tmp, BUFFER_SIZE);
		if (fd_ret == -1)
		{
			free(tmp);
			return (NULL);
		}
		tmp[fd_ret] = '\0';
		start_str = start_str_generator(start_str, tmp);
	}
	free(tmp);
	tmp = trim_line(start_str);
	start_str = trim_buffer(start_str);
	return (tmp);
}
