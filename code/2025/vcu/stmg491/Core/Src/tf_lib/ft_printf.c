/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_printf.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: urmet <urmet@student.42.fr>                +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2024/12/09 20:35:03 by cari              #+#    #+#             */
/*   Updated: 2025/02/25 13:28:56 by urmet            ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static void	print_base(unsigned long n, char *base, int *len, char *format)
{
	if (*format == 'd' || *format == 'i')
	{
		if ((long) n < 0)
		{
			*len += write(1, "-", 1);
			n *= -1;
		}
	}
	if (*format == 'p')
	{
		if (n == 0)
		{
			*len += write(1, "(nil)", 5);
			return ;
		}
		*len += write(1, "0x", 2);
	}
	if (n >= ft_strlen(base))
	{
		print_base(n / ft_strlen(base), base, len, "0");
		print_base(n % ft_strlen(base), base, len, "0");
	}
	else
		*len += write(1, &base[n], 1);
}

static void	print_str(char *str, int *len)
{
	if (!str)
		str = "(null)";
	*len += write(1, str, ft_strlen(str));
}

static void	ft_format_control(char *format, va_list arg, int *len)
{
	char	c;

	if (*format == 'c')
	{
		c = va_arg(arg, int);
		*len += write(1, &c, 1);
	}
	else if (*format == 's')
		print_str(va_arg(arg, char *), len);
	else if (*format == 'p')
		print_base(va_arg(arg, unsigned long), "0123456789abcdef", len, format);
	else if (*format == 'd' || *format == 'i')
		print_base(va_arg(arg, int), "0123456789", len, format);
	else if (*format == 'u')
		print_base(va_arg(arg, unsigned int), "0123456789", len, format);
	else if (*format == 'x')
		print_base(va_arg(arg, unsigned int), "0123456789abcdef", len, format);
	else if (*format == 'X')
		print_base(va_arg(arg, unsigned int), "0123456789ABCDEF", len, format);
	else if (*format == '%')
		*len += write(1, "%", 1);
}

int	ft_printf(const char *format, ...)
{
	va_list	args;
	int		len;
	int		*ptr;

	va_start(args, format);
	len = 0;
	ptr = &len;
	while (*format)
	{
		if (*format == '%')
		{
			if (!*(format + 1))
				return (0);
			ft_format_control((char *)++format, args, ptr);
		}
		else
			*ptr += write(1, format, 1);
		format++;
	}
	va_end(args);
	return (*ptr);
}
